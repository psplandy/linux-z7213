/*
 * (C) Copyright 2012
 *     wangflord@allwinnertech.com
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program;
 *
 */
#ifndef __boot0_v2_h
#define __boot0_v2_h

#define STAMP_VALUE                     0x5F0A6C39

typedef struct {
	__u8        ChipCnt;                            /*the count of the total nand flash chips are currently connecting on the CE pin*/
	__u16       ChipConnectInfo;                    /*chip connect information, bit == 1 means there is a chip connecting on the CE pin*/
	__u8        ConnectMode;						/*the rb connect  mode*/
	__u8        BankCntPerChip;                     /*the count of the banks in one nand chip, multiple banks can support Inter-Leave*/
	__u8        DieCntPerChip;                      /*the count of the dies in one nand chip, block management is based on Die*/
	__u8        PlaneCntPerDie;                     /*the count of planes in one die, multiple planes can support multi-plane operation*/
	__u8        SectorCntPerPage;                   /*the count of sectors in one single physic page, one sector is 0.5k*/
	__u16       PageCntPerPhyBlk;                   /*the count of physic pages in one physic block*/
	__u32       BlkCntPerDie;                       /*the count of the physic blocks in one die, include valid block and invalid block*/
	__u32       OperationOpt;                       /*the mask of the operation types which current nand flash can support support*/
	__u16       FrequencePar;                       /*the parameter of the hardware access clock, based on 'MHz'*/
	__u32       SpiMode;                            /*spi nand mode, 0:mode 0, 3:mode 3*/
	__u8        NandChipId[8];                      /*the nand chip id of current connecting nand chip*/
	__u32		pagewithbadflag;					/*bad block flag was written at the first byte of spare area of this page*/
	__u32       MultiPlaneBlockOffset;              /*the value of the block number offset between the two plane block*/
	__u32       MaxEraseTimes;              		/*the max erase times of a physic block*/
	__u32		MaxEccBits;							/*the max ecc bits that nand support*/
	__u32		EccLimitBits;						/*the ecc limit flag for tne nand*/
	__u32		Reserved[4];
} boot_nand_para_t0;


/*ͨ�õģ���GPIO��ص����ݽṹ*/
typedef struct _normal_gpio_cfg {
	unsigned char      port;                       /*�˿ں�*/
	unsigned char      port_num;                   /*�˿��ڱ��*/
	char      mul_sel;                    /*���ܱ��*/
	char      pull;                       /*����״̬*/
	char      drv_level;                  /*������������*/
	char      data;                       /*�����ƽ*/
	unsigned char      reserved[2];                /*����λ����֤����*/
}
normal_gpio_cfg;

/******************************************************************************/
/*                              file head of Boot0                            */
/******************************************************************************/
typedef struct _boot0_private_head_t {
	unsigned int            prvt_head_size;
	char                    prvt_head_vsn[4];       /* the version of boot0_private_head_t*/
	unsigned int            dram_para[32];          /* DRAM patameters for initialising dram. Original values is arbitrary,*/
	int						uart_port;              /* UART���������*/
	normal_gpio_cfg         uart_ctrl[2];           /* UART������(���Դ�ӡ��)������Ϣ*/
	int                     enable_jtag;            /* 1 : enable,  0 : disable*/
	normal_gpio_cfg	        jtag_gpio[5];           /* ����JTAG��ȫ��GPIO��Ϣ*/
	normal_gpio_cfg         storage_gpio[32];       /* �洢�豸 GPIO��Ϣ*/
	char                    storage_data[512 - sizeof(normal_gpio_cfg) * 32];      /* �û�����������Ϣ*/
	/*boot_nand_connect_info_t    nand_connect_info;*/
} boot0_private_head_t;


typedef struct standard_Boot_file_head {
	unsigned int  jump_instruction;   /* one intruction jumping to real code*/
	unsigned char   magic[8];           /* ="eGON.BT0" or "eGON.BT1",  not C-style string.*/
	unsigned int  check_sum;          /* generated by PC*/
	unsigned int  length;             /* generated by PC*/
	unsigned int  pub_head_size;      /* the size of boot_file_head_t*/
	unsigned char   pub_head_vsn[4];    /* the version of boot_file_head_t*/
	unsigned char   file_head_vsn[4];   /* the version of boot0_file_head_t or boot1_file_head_t*/
	unsigned char   Boot_vsn[4];        /* Boot version*/
	unsigned char   eGON_vsn[4];        /* eGON version*/
	unsigned char   platform[8];        /* platform information*/
} standard_boot_file_head_t;


typedef struct _boot0_file_head_t {
	standard_boot_file_head_t   boot_head;
	boot0_private_head_t  		prvt_head;
} boot0_file_head_t;

typedef struct sbrom_toc0_config {
	unsigned char    	config_vsn[4];
	unsigned int      	dram_para[32];  	/* dram����*/
	int				  	uart_port;      	/* UART���������*/
	normal_gpio_cfg   	uart_ctrl[2];    	/* UART������GPIO*/
	int              	enable_jtag;    	/* JTAGʹ��*/
	normal_gpio_cfg   	jtag_gpio[5];    	/* JTAG������GPIO*/
	normal_gpio_cfg  	storage_gpio[50]; 	/* �洢�豸 GPIO��Ϣ*/
	/* 0-23��nand��24-31��ſ�0��32-39�ſ�2*/
	/* 40-49���spi*/
	char   				storage_data[384];  /* 0-159,�洢nand��Ϣ��160-255,��ſ���Ϣ*/
	unsigned int        secure_dram_mbytes; /**/
	unsigned int        drm_start_mbytes;   /**/
	unsigned int        drm_size_mbytes;    /**/
	unsigned int      	res[8];   			/* �ܹ�1024�ֽ�*/
} sbrom_toc0_config_t;

typedef struct {
	u8  name[8];	  /*�ַ��������Ը��ģ�û��������*/
	u32 magic;	      /*������0x89119800*/
	u32 check_sum;    /*�������ݵ�У��ͣ��ο�����boot0����*/

	u32 serial_num;   /*���кţ����Ը��ģ�û������*/
	u32 status;       /*���Ը��ģ�û������*/

	u32 items_nr;	  /*�ܵ���Ŀ��������TOC0��˵��������2*/
	u32 length;	      /*TOC0�ĳ���*/
	u8  platform[4];  /*toc_platform[0]��ʾ�������*/
	/*0��nand��1����0��2����2��3��spinor*/
	u32 reserved[2];  /*����λ*/
	u32 end;          /*��ʾͷ���ṹ�������������0x3b45494d*/

} toc0_private_head_t;
#define SBROM_TOC0_HEAD_SPACE 0x80






#endif     /*  ifndef __boot0_h*/

/* end of boot0.h */
