/*
 * MPS MP3309 white-LED backlight in I2C (analog) dimming mode, for the
 * Meraki MC74 (i2c-2 @ 0x17). u-boot leaves it at 0xfc (on, full level).
 * Register 0: EN | D0 | D1 | D2 | D3 | D4 | x | x  (D0 is the LSB).
 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/slab.h>
#include <linux/err.h>

#define MP3309_REG0		0x00
#define MP3309_REG1		0x01
/* reg1: SYNC | OVP1, DIMS clear = I2C (analog) dimming; DIMS set lets the
 * VideoCore PWM dim the LEDs, which flickers on every screen update. */
#define MP3309_REG1_VAL		0x28
#define MP3309_EN		0x80
#define MP3309_LEVELS		32

struct mp3309 {
	struct i2c_client *client;
	struct backlight_device *bl;
};

static int mp3309_update_status(struct backlight_device *bl)
{
	struct mp3309 *chip = bl_get_data(bl);
	int brightness = bl->props.brightness;
	int level, i, val = 0;

	if (bl->props.power != FB_BLANK_UNBLANK || bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	level = (brightness * (MP3309_LEVELS - 1) + 127) / 255;
	for (i = 0; i < 5; i++)
		val |= ((level >> i) & 1) << (6 - i);
	if (brightness)
		val |= MP3309_EN;

	return i2c_smbus_write_byte_data(chip->client, MP3309_REG0, val);
}

static int mp3309_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static const struct backlight_ops mp3309_ops = {
	.update_status = mp3309_update_status,
	.get_brightness = mp3309_get_brightness,
};

static int mp3309_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct backlight_properties props;
	struct mp3309 *chip;
	int reg;

	reg = i2c_smbus_read_byte_data(client, MP3309_REG0);
	if (reg < 0) {
		dev_err(&client->dev, "no MP3309 at 0x%02x: %d\n", client->addr, reg);
		return reg;
	}

	{
		int r1 = i2c_smbus_read_byte_data(client, MP3309_REG1);

		i2c_smbus_write_byte_data(client, MP3309_REG1, MP3309_REG1_VAL);
		dev_info(&client->dev, "reg1 0x%02x -> 0x%02x (I2C dimming)\n", r1, MP3309_REG1_VAL);
	}

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	chip->client = client;

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = 255;
	props.brightness = 255;	/* matches what u-boot left */
	chip->bl = backlight_device_register("mp3309", &client->dev, chip,
					     &mp3309_ops, &props);
	if (IS_ERR(chip->bl)) {
		int err = PTR_ERR(chip->bl);
		kfree(chip);
		return err;
	}
	i2c_set_clientdata(client, chip);
	dev_info(&client->dev, "MP3309 backlight, reg0 was 0x%02x\n", reg);
	return 0;
}

static int mp3309_remove(struct i2c_client *client)
{
	struct mp3309 *chip = i2c_get_clientdata(client);

	backlight_device_unregister(chip->bl);
	kfree(chip);
	return 0;
}

static const struct i2c_device_id mp3309_id[] = {
	{ "mp3309", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mp3309_id);

static struct i2c_driver mp3309_driver = {
	.driver = { .name = "mp3309", .owner = THIS_MODULE },
	.probe = mp3309_probe,
	.remove = mp3309_remove,
	.id_table = mp3309_id,
};

static int __init mp3309_init(void)
{
	return i2c_add_driver(&mp3309_driver);
}
module_init(mp3309_init);

MODULE_LICENSE("GPL");
