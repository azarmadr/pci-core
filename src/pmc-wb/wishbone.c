#include <linux/module.h>

#include <linux/fs.h>
#include <linux/aio.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/socket.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/version.h>
#include <linux/slab.h>

#include "wishbone.h"

/* Module parameters */
static unsigned int max_devices = WISHONE_MAX_DEVICES;

/* Module globals */
static LIST_HEAD(wishbone_list); /* Sorted by ascending minor number */
static DEFINE_MUTEX(wishbone_mutex);
static struct class *wishbone_master_class;
static struct class *wishbone_slave_class;
static dev_t wishbone_master_dev_first;
static dev_t wishbone_slave_dev_first;

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,30)

/* missing 'const' in 2.6.30. present in 2.6.31. */
static int compat_memcpy_fromiovecend(unsigned char *kdata, const struct iovec *iov,
                        int offset, int len)
{
        /* Skip over the finished iovecs */
        while (offset >= iov->iov_len) {
                offset -= iov->iov_len;
                iov++;
        }

        while (len > 0) {
                u8 __user *base = iov->iov_base + offset;
                int copy = min_t(unsigned int, len, iov->iov_len - offset);

                offset = 0;
                if (copy_from_user(kdata, base, copy))
                        return -EFAULT;
                len -= copy;
                kdata += copy;
                iov++;
        }

        return 0;
}


/* does not exist in 2.6.30. does in 2.6.31. */
static int compat_memcpy_toiovecend(const struct iovec *iov, unsigned char *kdata,
                       int offset, int len)
 {
         int copy;
         for (; len > 0; ++iov) {
                 /* Skip over the finished iovecs */
                 if (unlikely(offset >= iov->iov_len)) {
                         offset -= iov->iov_len;
                         continue;
                 }
                 copy = min_t(unsigned int, iov->iov_len - offset, len);
                 if (copy_to_user(iov->iov_base + offset, kdata, copy))
                         return -EFAULT;
                 offset = 0;
                 kdata += copy;
                 len -= copy;
         }

         return 0;
}

/* Over-ride with compatible versions */
#define memcpy_toiovecend   compat_memcpy_toiovecend
#define memcpy_fromiovecend compat_memcpy_fromiovecend
#endif

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,26)
/* Older linux versions do not have the drvdata 'a' parameter. >= 2.6.37 present. */
#define device_create(c, p, d, a, f, x) device_create(c, p, d, f, x)
#endif

/* Compiler should be able to optimize this to one inlined instruction */
static inline wb_data_t eb_to_cpu(unsigned char* x)
{
	switch (sizeof(wb_data_t)) {
	case 8: return be64_to_cpu(*(wb_data_t*)x);
	case 4: return be32_to_cpu(*(wb_data_t*)x);
	case 2: return be16_to_cpu(*(wb_data_t*)x);
	case 1: return *(wb_data_t*)x;
	}
}

/* Compiler should be able to optimize this to one inlined instruction */
static inline void eb_from_cpu(unsigned char* x, wb_data_t dat)
{
	switch (sizeof(wb_data_t)) {
	case 8: *(wb_data_t*)x = cpu_to_be64(dat); break;
	case 4: *(wb_data_t*)x = cpu_to_be32(dat); break;
	case 2: *(wb_data_t*)x = cpu_to_be16(dat); break;
	case 1: *(wb_data_t*)x = dat;              break;
	}
}

static void etherbone_master_process(struct etherbone_master_context* context)
{
	struct wishbone *wb;
	const struct wishbone_operations *wops;
	unsigned int size, left, i, record_len;
	unsigned char *buf;
	
	if (context->state == header) {
		if (context->received < 8) {
			/* no-op */
			return;
		}
		
		context->buf[0] = 0x4E;
		context->buf[1] = 0x6F;
		context->buf[2] = 0x12; /* V.1 Probe-Response */
		context->buf[3] = (sizeof(wb_addr_t)<<4) | sizeof(wb_data_t);
		/* Echo back bytes 4-7, the probe identifier */
		context->processed = 8;
		context->state = idle;
	}
	
	buf = &context->buf[0];
	wb = context->wishbone;
	wops = wb->wops;
	
	i = RING_INDEX(context->processed);
	size = RING_PROC_LEN(context);
	
	for (left = size; left >= 4; left -= record_len) {
		unsigned char flags, be, wcount, rcount;
		
		/* Determine record size */
		flags  = buf[i+0];
		be     = buf[i+1];
		wcount = buf[i+2];
		rcount = buf[i+3];
		
		record_len = 1 + wcount + rcount + (wcount > 0) + (rcount > 0);
		record_len *= sizeof(wb_data_t);
		
		if (left < record_len) break;
		
		/* Configure byte enable and raise cycle line */
		if (context->state == idle) {
			wops->cycle(wb, 1);
			context->state = cycle;
		}
		wops->byteenable(wb, be);

		/* Process the writes */
		if (wcount > 0) {
			wb_addr_t base_address, increment;
			unsigned char j;
			int wff = flags & ETHERBONE_WFF;
			int wca = flags & ETHERBONE_WCA;
			
			/* increment=0 if wff!=0 */
			increment = sizeof(wb_data_t) * (1 - (wff / ETHERBONE_WFF));
			
			/* Erase the header */
			eb_from_cpu(buf+i, 0);
			i = RING_INDEX(i + sizeof(wb_data_t));
			base_address = eb_to_cpu(buf+i);
			
			if (wca) {
				for (j = wcount; j > 0; --j) {
					eb_from_cpu(buf+i, 0);
					i = RING_INDEX(i + sizeof(wb_data_t));
				}
			} else {
				for (j = wcount; j > 0; --j) {
					eb_from_cpu(buf+i, 0);
					i = RING_INDEX(i + sizeof(wb_data_t));
					wops->write(wb, base_address, eb_to_cpu(buf+i));
					base_address += increment;
				}
			}
		}
		
		buf[i+0] = (flags & ETHERBONE_CYC) | 
		           (((flags & ETHERBONE_RFF) != 0) ? ETHERBONE_WFF : 0) |
		           (((flags & ETHERBONE_BCA) != 0) ? ETHERBONE_WCA : 0);
		buf[i+1] = be;
		buf[i+2] = rcount; /* rcount -> wcount */
		buf[i+3] = 0;
		
		if (rcount > 0) {
			unsigned char j;
			int rca = flags & ETHERBONE_RCA;
			
			/* Move past header, and leave BaseRetAddr intact */
			i = RING_INDEX(i + sizeof(wb_data_t) + sizeof(wb_data_t));
			
			if (rca) {
				for (j = rcount; j > 0; --j) {
					eb_from_cpu(buf+i, wops->read_cfg(wb, eb_to_cpu(buf+i)));
					i = RING_INDEX(i + sizeof(wb_data_t));
				}
			} else {
				for (j = rcount; j > 0; --j) {
					eb_from_cpu(buf+i, wops->read(wb, eb_to_cpu(buf+i)));
					i = RING_INDEX(i + sizeof(wb_data_t));
				}
			}
		} else {
			i = RING_INDEX(i + sizeof(wb_data_t));
		}
		
		if ((flags & ETHERBONE_CYC) != 0) {
			wops->cycle(wb, 0);
			context->state = idle;
		}
	}
	
	context->processed = RING_POS(context->processed + size - left);
}

static void etherbone_slave_out_process(struct etherbone_slave_context *context)
{
	struct wishbone_request request;
	uint8_t *wptr;
	
	if (context->rbuf_done != context->rbuf_end ||   /* unread data? */
	    !context->negotiated ||                      /* negotiation incomplete? */
	    !context->wishbone->wops->request(context->wishbone, &request)) {
		return;
	}
	
	++context->pending_err;
	
	context->rbuf_done = 0;
	
	wptr = &context->rbuf[0];
	
	wptr[0] = ETHERBONE_BCA;
	wptr[1] = request.mask;
	
	if (request.write) {
		wptr[2] = 1;
		wptr[3] = 0;
		wptr += sizeof(wb_data_t);
		eb_from_cpu(wptr, request.addr);
		wptr += sizeof(wb_data_t);
		eb_from_cpu(wptr, request.data);
		wptr += sizeof(wb_data_t); 
	} else {
		wptr[2] = 0;
		wptr[3] = 1;
		wptr += sizeof(wb_data_t); 
		eb_from_cpu(wptr, WBA_DATA);
		wptr += sizeof(wb_data_t); 
		eb_from_cpu(wptr, request.addr);
		wptr += sizeof(wb_data_t); 
	}
	
	wptr[0] = ETHERBONE_CYC /* !!! */ | ETHERBONE_BCA | ETHERBONE_RCA;
	wptr[1] = 0xf;
	wptr[2] = 0;
	wptr[3] = 1;
	wptr += sizeof(wb_data_t); 
	
	eb_from_cpu(wptr, WBA_ERR);
	wptr += sizeof(wb_data_t); 
	eb_from_cpu(wptr, 4); /* low bits of error status register */
	wptr += sizeof(wb_data_t); 
	
	context->rbuf_end = wptr - &context->rbuf[0];
}

static int etherbone_slave_in_process(struct etherbone_slave_context *context)
{
	struct wishbone *wb;
	const struct wishbone_operations *wops;
	unsigned char flags, be, wcount, rcount;
	uint8_t *rptr;
	wb_addr_t base_address;
	wb_data_t data;
	unsigned char j;
	int wff;
	
	wb = context->wishbone;
	wops = wb->wops;
	
	/* Process record header */
	rptr = &context->wbuf[0];
	flags  = rptr[0];
	be     = rptr[1];
	wcount = rptr[2];
	rcount = rptr[3];
	rptr += sizeof(wb_data_t);
	
	/* Must be a full write to the config space! */
	if (rcount != 0) return -EIO;
	if (wcount == 0) return 0;
	
	if (be != 0xf) return -EIO;
	if ((flags & ETHERBONE_WCA) == 0) return -EIO;
	
	/* Process the writes */
	base_address = eb_to_cpu(rptr);
	rptr += sizeof(wb_data_t);
	wff = flags & ETHERBONE_WFF;
	
	for (j = 0; j < wcount; ++j) {
		data = eb_to_cpu(rptr);
		rptr += sizeof(wb_data_t);
		
		switch (base_address) {
		case WBA_DATA:
			context->data = data;
			break;
		case WBA_ERR:
			if (context->pending_err == 0) return -EIO;
			--context->pending_err;
			wops->reply(wb, data&1, context->data);
			break;
		default:
			return -EIO;
		}
		
		if (!wff) base_address += sizeof(wb_data_t);
	}
	
	return 0;
}

static int char_master_open(struct inode *inode, struct file *filep)
{	
	struct etherbone_master_context *context;
	
	context = kmalloc(sizeof(struct etherbone_master_context), GFP_KERNEL);
	if (!context) return -ENOMEM;
	
	context->wishbone = container_of(inode->i_cdev, struct wishbone, master_cdev);
	context->fasync = 0;
	mutex_init(&context->mutex);
	init_waitqueue_head(&context->waitq);
	context->state = header;
	context->sent = 0;
	context->processed = 0;
	context->received = 0;
	
	filep->private_data = context;
	
	return 0;
}

static int char_master_release(struct inode *inode, struct file *filep)
{
	struct etherbone_master_context *context = filep->private_data;
	
	/* Did the bad user forget to drop the cycle line? */
	if (context->state == cycle) {
		context->wishbone->wops->cycle(context->wishbone, 0);
	}
	
	kfree(context);
	return 0;
}

static ssize_t char_master_aio_read(struct kiocb *iocb, const struct iovec *iov, unsigned long nr_segs, loff_t pos)
{
	struct file *filep = iocb->ki_filp;
	struct etherbone_master_context *context = filep->private_data;
	unsigned int len, iov_len, ring_len, buf_len;
	
	iov_len = iov_length(iov, nr_segs);
	if (unlikely(iov_len == 0)) return 0;
	
	if (mutex_lock_interruptible(&context->mutex))
		return -EINTR;
	
	ring_len = RING_READ_LEN(context);
	len = min_t(unsigned int, ring_len, iov_len);
	
	/* How far till we must wrap?  */
	buf_len = sizeof(context->buf) - RING_INDEX(context->sent);
	
	if (buf_len < len) {
		memcpy_toiovecend(iov, RING_POINTER(context, sent), 0, buf_len);
		memcpy_toiovecend(iov, &context->buf[0],            buf_len, len-buf_len);
	} else {
		memcpy_toiovecend(iov, RING_POINTER(context, sent), 0, len);
	}
	context->sent = RING_POS(context->sent + len);
	
	mutex_unlock(&context->mutex);
	
	/* Wake-up polling descriptors */
	wake_up_interruptible(&context->waitq);
	kill_fasync(&context->fasync, SIGIO, POLL_OUT);
	
	if (len == 0 && (filep->f_flags & O_NONBLOCK) != 0)
		return -EAGAIN;
	
	return len;
}

static ssize_t char_master_aio_write(struct kiocb *iocb, const struct iovec *iov, unsigned long nr_segs, loff_t pos)
{
	struct file *filep = iocb->ki_filp;
	struct etherbone_master_context *context = filep->private_data;
	unsigned int len, iov_len, ring_len, buf_len;
	
	iov_len = iov_length(iov, nr_segs);
	if (unlikely(iov_len == 0)) return 0;
	
	if (mutex_lock_interruptible(&context->mutex))
		return -EINTR;
	
	ring_len = RING_WRITE_LEN(context);
	len = min_t(unsigned int, ring_len, iov_len);
	
	/* How far till we must wrap?  */
	buf_len = sizeof(context->buf) - RING_INDEX(context->received);
	
	if (buf_len < len) {
		memcpy_fromiovecend(RING_POINTER(context, received), iov, 0, buf_len);
		memcpy_fromiovecend(&context->buf[0],                iov, buf_len, len-buf_len);
	} else {
		memcpy_fromiovecend(RING_POINTER(context, received), iov, 0, len);
	}
	context->received = RING_POS(context->received + len);
	
	/* Process buffers */
	etherbone_master_process(context);
	
	mutex_unlock(&context->mutex);
	
	/* Wake-up polling descriptors */
	wake_up_interruptible(&context->waitq);
	kill_fasync(&context->fasync, SIGIO, POLL_IN);
	
	if (len == 0 && (filep->f_flags & O_NONBLOCK) != 0)
		return -EAGAIN;
	
	return len;
}

static unsigned int char_master_poll(struct file *filep, poll_table *wait)
{
	unsigned int mask = 0;
	struct etherbone_master_context *context = filep->private_data;
	
	poll_wait(filep, &context->waitq, wait);
	
	mutex_lock(&context->mutex);
	
	if (RING_READ_LEN (context) != 0) mask |= POLLIN  | POLLRDNORM;
	if (RING_WRITE_LEN(context) != 0) mask |= POLLOUT | POLLWRNORM;
	
	mutex_unlock(&context->mutex);
	
	return mask;
}

static int char_master_fasync(int fd, struct file *file, int on)
{
	struct etherbone_master_context* context = file->private_data;

        /* No locking - fasync_helper does its own locking */
        return fasync_helper(fd, file, on, &context->fasync);
}

static const struct file_operations etherbone_master_fops = {
        .owner          = THIS_MODULE,
        .llseek         = no_llseek,
        .read           = do_sync_read,
        .aio_read       = char_master_aio_read,
        .write          = do_sync_write,
        .aio_write      = char_master_aio_write,
        .open           = char_master_open,
        .poll           = char_master_poll,
        .release        = char_master_release,
        .fasync         = char_master_fasync,
};

void wishbone_slave_ready(struct wishbone* wb)
{
	wake_up_interruptible(&wb->waitq);
	kill_fasync(&wb->fasync, SIGIO, POLL_IN);
}

static int char_slave_open(struct inode *inode, struct file *filep)
{	
	struct etherbone_slave_context *context;
	
	context = kmalloc(sizeof(struct etherbone_slave_context), GFP_KERNEL);
	if (!context) return -ENOMEM;
	
	context->wishbone = container_of(inode->i_cdev, struct wishbone, slave_cdev);
	mutex_init(&context->mutex);
	
	filep->private_data = context;
	
	/* Only a single open of the slave device is allowed */
	mutex_lock(&context->wishbone->mutex);
	if (context->wishbone->slave) {
		mutex_unlock(&context->wishbone->mutex);
		kfree(context);
		return -EBUSY;
	}
	context->wishbone->slave = context;
	
	/* Setup Etherbone state machine */
	context->pending_err = 0;
	context->negotiated = 0;
	context->rbuf_done = 0;
	context->wbuf_fill = 0;
	
	/* Fill in the EB request */
	context->rbuf[0] = 0x4E;
	context->rbuf[1] = 0x6F;
	context->rbuf[2] = 0x11; /* V1 probe */
	context->rbuf[3] = 0x44; /* 32-bit only */
	memset(&context->rbuf[4], 0, 4);
	context->rbuf_end = 8;
	
	mutex_unlock(&context->wishbone->mutex);
	
	return 0;
}

static int char_slave_release(struct inode *inode, struct file *filep)
{
	struct etherbone_slave_context *context = filep->private_data;
	const struct wishbone_operations *wops = context->wishbone->wops;
	int pending, errors;
	
	mutex_lock(&context->mutex);
	
	/* We need to answer any requests pending, even if the bad user did not */
	errors = context->pending_err;
	for (pending = errors; pending > 0; --pending)
		wops->reply(context->wishbone, 1, 0);
	
	mutex_lock(&context->wishbone->mutex);
	context->wishbone->slave = 0;
	mutex_unlock(&context->wishbone->mutex);
	
	mutex_unlock(&context->mutex);
	
	kfree(context);
	
	if (errors) return -EIO;
	return 0;
}

static ssize_t char_slave_aio_read(struct kiocb *iocb, const struct iovec *iov, unsigned long nr_segs, loff_t pos)
{
	struct file *filep = iocb->ki_filp;
	struct etherbone_slave_context *context = filep->private_data;
	unsigned int iov_len, buf_len, len;
	
	iov_len = iov_length(iov, nr_segs);
	if (unlikely(iov_len == 0)) return 0;
	
	if (mutex_lock_interruptible(&context->mutex))
		return -EINTR;
	
	etherbone_slave_out_process(context);
	
	buf_len = context->rbuf_end - context->rbuf_done;
	if (buf_len > iov_len) {
		len = iov_len;
	} else {
		len = buf_len;
	}
	
	memcpy_toiovecend(iov, context->rbuf + context->rbuf_done, 0, len);
	context->rbuf_done += len;
	
	mutex_unlock(&context->mutex);
	
	if (len == 0 && (filep->f_flags & O_NONBLOCK) != 0)
		return -EAGAIN;
	
	return len;
}

static ssize_t char_slave_aio_write(struct kiocb *iocb, const struct iovec *iov, unsigned long nr_segs, loff_t pos)
{
	struct file *filep = iocb->ki_filp;
	struct etherbone_slave_context *context = filep->private_data;
	unsigned int iov_len, iov_off, buf_len, len, outlen;
	
	outlen = iov_length(iov, nr_segs);
	
	if (mutex_lock_interruptible(&context->mutex))
		return -EINTR;
	
	for (iov_off = 0, iov_len = outlen; iov_len > 0; iov_off += len, iov_len -= len) {
		/* Optimization to avoid quadratic complexity */
		while (iov_off >= iov->iov_len) {
			iov_off -= iov->iov_len;
			++iov;
		}
		
		if (context->wbuf_fill < 4) {
			buf_len = 4 - context->wbuf_fill;
			if (buf_len > iov_len) {
				len = iov_len;
			} else {
				len = buf_len;
			}
			
			memcpy_fromiovecend(context->wbuf + context->wbuf_fill, iov, iov_off, len);
			context->wbuf_fill += len;
		} else {
			if (!context->negotiated) {
				if (context->wbuf[0] != 0x4E ||
				    context->wbuf[1] != 0x6F ||
				    (context->wbuf[2] & 0x7) != 0x6 ||
				    (context->wbuf[3] & 0x44) != 0x44) {
					break;
				} else {
					context->wbuf_fill = 0;
					context->negotiated = 1;
					wake_up_interruptible(&context->wishbone->waitq);
				        kill_fasync(&context->wishbone->fasync, SIGIO, POLL_IN);
				}
				len = 0;
			} else {
				buf_len = 
					((context->wbuf[2] > 0) +
					 (context->wbuf[3] > 0) +
					 context->wbuf[2] +
					 context->wbuf[3] +
					 1) * 4;
				buf_len -= context->wbuf_fill;
				
				if (buf_len > iov_len) {
					len = iov_len;
					memcpy_fromiovecend(context->wbuf + context->wbuf_fill, iov, iov_off, len);
					context->wbuf_fill += len;
				} else {
					len = buf_len;
					memcpy_fromiovecend(context->wbuf + context->wbuf_fill, iov, iov_off, len);
					context->wbuf_fill = 0;
					if (etherbone_slave_in_process(context) != 0) break;
				}
			}
		}
	}
	
	mutex_unlock(&context->mutex);
	
	if (iov_len > 0) return -EIO;
	return outlen;
}

static unsigned int char_slave_poll(struct file *filep, poll_table *wait)
{
	struct etherbone_slave_context *context = filep->private_data;
	unsigned int mask;
	
	poll_wait(filep, &context->wishbone->waitq, wait);
	
	mutex_lock(&context->mutex);
	etherbone_slave_out_process(context);
	
	if (context->rbuf_done != context->rbuf_end) {
		mask = POLLIN  | POLLRDNORM | POLLOUT | POLLWRNORM;
	} else {
		mask = POLLOUT | POLLWRNORM;
	}
	
	mutex_unlock(&context->mutex);
	
	return mask;
}

static int char_slave_fasync(int fd, struct file *file, int on)
{
	struct etherbone_slave_context* context = file->private_data;

        /* No locking - fasync_helper does its own locking */
        return fasync_helper(fd, file, on, &context->wishbone->fasync);
}

static const struct file_operations etherbone_slave_fops = {
        .owner          = THIS_MODULE,
        .llseek         = no_llseek,
        .read           = do_sync_read,
        .aio_read       = char_slave_aio_read,
        .write          = do_sync_write,
        .aio_write      = char_slave_aio_write,
        .open           = char_slave_open,
        .poll           = char_slave_poll,
        .release        = char_slave_release,
        .fasync         = char_slave_fasync,
};

int wishbone_register(struct wishbone* wb)
{
	struct list_head *list_pos;
	unsigned int devoff;
	
	INIT_LIST_HEAD(&wb->list);
	
	mutex_lock(&wishbone_mutex);
	
	/* Search the list for gaps, stopping past the gap.
	 * If we overflow the list (ie: not gaps), minor already points past end.
	 */
	devoff = 0;
	list_for_each(list_pos, &wishbone_list) {
		struct wishbone *entry =
			container_of(list_pos, struct wishbone, list);
		
		dev_t master_dev_tmp = 
		  MKDEV(
		    MAJOR(wishbone_master_dev_first),
		    MINOR(wishbone_master_dev_first) + devoff);
		
		if (entry->master_dev != master_dev_tmp) {
			/* We found a gap! */
			break;
		} else {
			/* Run out of minors? */
			if (devoff == max_devices-1) goto fail_out;
			
			/* Try the next minor */
			++devoff;
		}
	}
	
	init_waitqueue_head(&wb->waitq);
	wb->fasync = 0;
	mutex_init(&wb->mutex);
	wb->slave = 0;
	
	/* Connect the file operations with the cdevs */
	cdev_init(&wb->master_cdev, &etherbone_master_fops);
	wb->master_cdev.owner = wb->wops->owner;
	
	cdev_init(&wb->slave_cdev,  &etherbone_slave_fops);
	wb->slave_cdev.owner = wb->wops->owner;
	
	wb->master_dev =
	  MKDEV(
	    MAJOR(wishbone_master_dev_first), 
	    MINOR(wishbone_master_dev_first) + devoff);
	wb->slave_dev = 
	  MKDEV(
	    MAJOR(wishbone_slave_dev_first), 
	    MINOR(wishbone_slave_dev_first) + devoff);
	
	/* Connect the major/minor number to the cdev */
	if (cdev_add(&wb->master_cdev, wb->master_dev, 1)) goto fail_out;
	if (cdev_add(&wb->slave_cdev,  wb->slave_dev,  1)) goto fail_master_cdev;
	
	/* Create the sysfs entry */
	wb->master_device = device_create(wishbone_master_class, wb->parent, wb->master_dev, NULL, "wbm%d", devoff);
	if (IS_ERR(wb->master_device)) goto fail_slave_cdev;
	wb->slave_device  = device_create(wishbone_slave_class,  wb->parent, wb->slave_dev,  NULL, "wbs%d", devoff);
	if (IS_ERR(wb->slave_device)) goto fail_master_sys;
	
	/* Insert the device into the gap */
	list_add_tail(&wb->list, list_pos);
	
	mutex_unlock(&wishbone_mutex);
	return 0;

fail_master_sys:
	device_destroy(wishbone_master_class, wb->master_dev);
fail_slave_cdev:
	cdev_del(&wb->slave_cdev);
fail_master_cdev:
	cdev_del(&wb->master_cdev);
fail_out:
	mutex_unlock(&wishbone_mutex);
	return -ENOMEM;
}

int wishbone_unregister(struct wishbone* wb)
{
	if (WARN_ON(list_empty(&wb->list)))
		return -EINVAL;
	
	mutex_lock(&wishbone_mutex);
	list_del(&wb->list);
	device_destroy(wishbone_slave_class,  wb->slave_dev);
	device_destroy(wishbone_master_class, wb->master_dev);
	cdev_del(&wb->slave_cdev);
	cdev_del(&wb->master_cdev);
	mutex_unlock(&wishbone_mutex);
	
	return 0;
}

static int __init wishbone_init(void)
{
	int err;
	dev_t overflow;

	printk(KERN_NOTICE "wishbone: version " __stringify(GIT_REVISION) " loaded\n");
	
	overflow = MKDEV(0, max_devices-1);
	if (MINOR(overflow) != max_devices-1) {
		err = -ENOMEM;
		goto fail_last;
	}
	
	wishbone_master_class = class_create(THIS_MODULE, "wbm");
	if (IS_ERR(wishbone_master_class)) {
		err = PTR_ERR(wishbone_master_class);
		goto fail_last;
	}
	
	wishbone_slave_class = class_create(THIS_MODULE, "wbs");
	if (IS_ERR(wishbone_slave_class)) {
		err = PTR_ERR(wishbone_slave_class);
		goto fail_master_class;
	}
	
	if (alloc_chrdev_region(&wishbone_master_dev_first, 0, max_devices, "wbm") < 0) {
		err = -EIO;
		goto fail_slave_class;
	}
	
	if (alloc_chrdev_region(&wishbone_slave_dev_first, 0, max_devices, "wbs") < 0) {
		err = -EIO;
		goto fail_master_dev;
	}
	
	return 0;

fail_master_dev:
	unregister_chrdev_region(wishbone_master_dev_first, max_devices);
fail_slave_class:
	class_destroy(wishbone_slave_class);
fail_master_class:
	class_destroy(wishbone_master_class);
fail_last:
	return err;
}

static void __exit wishbone_exit(void)
{
	unregister_chrdev_region(wishbone_slave_dev_first, max_devices);
	unregister_chrdev_region(wishbone_master_dev_first, max_devices);
	class_destroy(wishbone_slave_class);
	class_destroy(wishbone_master_class);
}

MODULE_AUTHOR("Wesley W. Terpstra <w.terpstra@gsi.de>");
MODULE_DESCRIPTION("Wishbone character device class");
module_param(max_devices, int, 0644);
MODULE_PARM_DESC(max_devices, "Maximum number of attached wishbone devices");
MODULE_LICENSE("GPL");
MODULE_VERSION(WISHBONE_VERSION);

EXPORT_SYMBOL(wishbone_register);
EXPORT_SYMBOL(wishbone_unregister);
EXPORT_SYMBOL(wishbone_slave_ready);

module_init(wishbone_init);
module_exit(wishbone_exit);
