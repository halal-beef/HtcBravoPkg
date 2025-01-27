/*
 * (C) Copyright 2000-2004
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#ifndef _PART_H
#define _PART_H

/* Interface types: */
#define IF_TYPE_UNKNOWN		0
#define IF_TYPE_IDE			1
#define IF_TYPE_SCSI		2
#define IF_TYPE_ATAPI		3
#define IF_TYPE_USB			4
#define IF_TYPE_DOC			5
#define IF_TYPE_MMC			6
#define IF_TYPE_SD			7
#define IF_TYPE_SATA		8

/* Part types */
#define PART_TYPE_UNKNOWN	0x00
#define PART_TYPE_MAC		0x01
#define PART_TYPE_DOS		0x02
#define PART_TYPE_ISO		0x03
#define PART_TYPE_AMIGA		0x04
#define PART_TYPE_EFI		0x05

typedef struct block_dev_desc {
	int				if_type;		/* type of the interface */
	int				dev;			/* device number */
	unsigned char	part_type;		/* partition type */
	unsigned char	target;			/* target SCSI ID */
	unsigned char	lun;			/* target LUN */
	unsigned char	type;			/* device type */
	unsigned char	removable;		/* removable device */
#ifdef CONFIG_LBA48
	unsigned char	lba48;			/* device can use 48bit addr (ATA/ATAPI v7) */
#endif
	unsigned long	lba;			/* number of blocks */
	unsigned long	blksz;			/* block size */
	char			vendor [40+1];	/* IDE model, SCSI Vendor */
	char			product[20+1];	/* IDE Serial no, SCSI product */
	char			revision[8+1];	/* firmware revision */
	unsigned long	(*block_read)(int dev, unsigned long start, unsigned long blkcnt, void *buffer);
	unsigned long	(*block_write)(int dev, unsigned long start, unsigned long blkcnt, const void *buffer);
	unsigned long   (*block_erase)(int dev, unsigned long start, unsigned long blkcnt);
	void			*priv;			/* driver private struct pointer */
} block_dev_desc_t;

/* device types */
#define DEV_TYPE_UNKNOWN	0xff	/* not connected */
#define DEV_TYPE_HARDDISK	0x00	/* harddisk */
#define DEV_TYPE_TAPE		0x01	/* Tape */
#define DEV_TYPE_CDROM		0x05	/* CD-ROM */
#define DEV_TYPE_OPDISK		0x07	/* optical disk */

/* Error number defines */
#define	ENODEV				19		/* No such device */

typedef struct disk_partition {
	unsigned long	start;		/* # of first block in partition	*/
	unsigned long	size;		/* number of blocks in partition	*/
	unsigned long	blksz;		/* block size in bytes			*/
	unsigned char	name[32];	/* partition name			*/
	unsigned char	type[32];	/* string type description		*/
} disk_partition_t;

#endif /* _PART_H */
