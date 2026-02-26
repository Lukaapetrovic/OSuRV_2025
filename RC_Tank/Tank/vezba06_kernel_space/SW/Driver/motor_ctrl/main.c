
#include <linux/module.h> // module_init(), module_exit()
#include <linux/fs.h> // file_operations
#include <linux/errno.h> // EFAULT
#include <linux/uaccess.h> // copy_from_user(), copy_to_user()

MODULE_LICENSE("Dual BSD/GPL");

#include "include/motor_ctrl.h"
#include "gpio.h"
#include "pwm.h"
#include "hw_pwm.h"
#include "sw_pwm.h"
#include "bldc.h"
#include "servo_fb.h"


#define FAKE_FEEDBACK 1

#ifdef WIDE_PWM
#define DEFUALT_THRESHOLD 500
#else
#define DEFUALT_THRESHOLD 75
#endif

#if MOTOR_CLTR__N_SERVO != (HW_PWM__N_CH+SW_PWM__N_CH)
#error "MOTOR_CLTR__N_SERVO wrong!"
#endif
#if MOTOR_CLTR__N_BLDC != BLDC__N_CH
#error "MOTOR_CLTR__N_BLDC wrong!"
#endif



static int motor_ctrl_open(struct inode *inode, struct file *filp) {
	return 0;
}

static int motor_ctrl_release(struct inode *inode, struct file *filp) {
	return 0;
}

static int16_t duty[MOTOR_CLTR__N_SERVO];

static ssize_t motor_ctrl_write(
	struct file* filp,
	const char *buf,
	size_t len,
	loff_t *f_pos
) {
	
	//TODO copy_from_user() -> duty
	
	//TODO bldc__set_dir()
	//TODO pwm__set_threshold(ch, abs_duty << 1);
	return 0;
}


static ssize_t motor_ctrl_read(
	struct file* filp,
	char* buf,
	size_t len,
	loff_t* f_pos
) {
	motor_ctrl__read_arg_fb_t a;
	
	//TODO duty -> a.pos_fb

	//TODO bldc__get_pulse_cnt() -> a.pulse_cnt_fb
	
	//TODO copy_to_user()
	return 0;
}


static long motor_ctrl_ioctl(
	struct file* filp,
	unsigned int cmd,
	unsigned long arg
) {
	motor_ctrl__ioctl_arg_moduo_t ia;
	ia = *(motor_ctrl__ioctl_arg_moduo_t*)&arg;
	
	switch(cmd){
		case IOCTL_MOTOR_CLTR_SET_MODUO:
			//TODO pwm__set_moduo() from arg over ia
			break;
		default:
			break;
	}

	return 0;
}

loff_t motor_ctrl_llseek(
	struct file* filp,
	loff_t offset,
	int whence
) {
	switch(whence){
		case SEEK_SET:
			filp->f_pos = offset;
			break;
		case SEEK_CUR:
			filp->f_pos += offset;
			break;
		case SEEK_END:
			return -ENOSYS; // Function not implemented.
		default:
			return -EINVAL;
		}
	return filp->f_pos;
}

static struct file_operations motor_ctrl_fops = {
	open           : motor_ctrl_open,
	release        : motor_ctrl_release,
	read           : motor_ctrl_read,
	write          : motor_ctrl_write,
	unlocked_ioctl : motor_ctrl_ioctl,
	llseek         : motor_ctrl_llseek
};


void motor_ctrl_exit(void) {
	servo_fb__exit();
	bldc__exit();
	pwm__exit();
	gpio__exit();
	unregister_chrdev(DEV_MAJOR, DEV_NAME);
	
	printk(KERN_INFO DEV_NAME": Module removed.\n");
}

int motor_ctrl_init(void) {
	int r;
	uint8_t ch;

	printk(KERN_INFO DEV_NAME": Inserting module...\n");
	
	r = register_chrdev(DEV_MAJOR, DEV_NAME, &motor_ctrl_fops);
	if(r < 0){
		printk(KERN_ERR DEV_NAME": cannot obtain major number %d!\n", DEV_MAJOR);
		goto exit;
	}

	r = gpio__init();
	if(r){
		goto exit;
	}
	
	r = pwm__init();
	if(r){
		goto exit;
	}
	
	// 10us*2000 -> 20ms.
	for(ch = 0; ch < MOTOR_CLTR__N_SERVO; ch++){
		pwm__set_moduo(ch, 1000 << 1);
		pwm__set_threshold(ch, DEFUALT_THRESHOLD << 1);
	}
	
	r = bldc__init();
	if(r){
		goto exit;
	}
	
	r = servo_fb__init();
	if(r){
		goto exit;
	}
	
exit:
	if(r){
		printk(KERN_ERR DEV_NAME": %s() failed with %d!\n", __func__, r);
		motor_ctrl_exit();
	}else{
		printk(KERN_INFO DEV_NAME": Inserting module successful.\n");
	}
	return r;
}


module_init(motor_ctrl_init);
module_exit(motor_ctrl_exit);
