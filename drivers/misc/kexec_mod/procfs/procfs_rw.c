/**
 *  procfs2.c -  create a "file" in /proc
 *
 */

#include <linux/module.h>	/* Specifically, a module */
#include <linux/kernel.h>	/* We're doing kernel work */
#include <linux/proc_fs.h>	/* Necessary because we use the proc fs */
#include <asm/uaccess.h>	/* for copy_from_user */

#define PROCFS_MAX_SIZE		1024
#define PROCFS_NAME 		"atags"

MODULE_LICENSE("GPL");
/**
 * This structure hold information about the /proc file
 *
 */
static struct proc_dir_entry *atags;

/**
 * The buffer used to store character for this module
 *
 */
static char procfs_buffer[PROCFS_MAX_SIZE];

/**
 * The size of the buffer
 *
 */
static unsigned long procfs_buffer_size = 0;

/** 
 * This function is called then the /proc file is read
 *
 */
int 
procfile_read(char *buffer,
	      char **buffer_location,
	      off_t offset, int buffer_length, int *eof, void *data)
{
	int ret;
	
	printk(KERN_INFO "procfs_rw: procfile_read (/proc/%s) called\n", PROCFS_NAME);
	
	if (offset > 0) {
		/* we have finished to read, return 0 */
		ret  = 0;
	} else {
		/* fill the buffer, return the buffer size */
		memcpy(buffer, procfs_buffer, procfs_buffer_size);
		ret = procfs_buffer_size;
	}

	return ret;
}

/**
 * This function is called with the /proc file is written
 *
 */
int procfile_write(struct file *file, const char *buffer, unsigned long count,
		   void *data)
{
	/* get buffer size */
	procfs_buffer_size = count;
	if (procfs_buffer_size > PROCFS_MAX_SIZE ) {
		procfs_buffer_size = PROCFS_MAX_SIZE;
	}
	
	/* write data to the buffer */
	if ( copy_from_user(procfs_buffer, buffer, procfs_buffer_size) ) {
		return -EFAULT;
	}
	
	return procfs_buffer_size;
}

/**
 *This function is called when the module is loaded
 *
 */
int init_module()
{
	/* create the /proc file */
	atags = create_proc_entry(PROCFS_NAME, 0644, NULL);
	
	if (atags == NULL) {
		remove_proc_entry(PROCFS_NAME, NULL);
		printk(KERN_ALERT "Error: Could not initialize /proc/%s\n",
			PROCFS_NAME);
		return -ENOMEM;
	}

	atags->read_proc  = procfile_read;
	atags->write_proc = procfile_write;
	//atags->owner 	  = THIS_MODULE;
	atags->mode 	  = S_IFREG | S_IRUGO;
	atags->uid 	  = 0;
	atags->gid 	  = 0;
	atags->size 	  = 37;

	printk(KERN_INFO "procfs_rw: /proc/%s created\n", PROCFS_NAME);	
	return 0;	/* everything is ok */
}

/**
 *This function is called when the module is unloaded
 *
 */
void cleanup_module()
{
	remove_proc_entry(PROCFS_NAME, NULL);
	printk(KERN_INFO "procfs_rw: /proc/%s removed\n", PROCFS_NAME);
}
