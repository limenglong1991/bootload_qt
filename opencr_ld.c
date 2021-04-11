#include "opencr_ld.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#include <stdarg.h>

#include "serial.h"
#include "type.h"
#include "./msg/msg.h"
//#include <sys/time.h>

#include <time.h>
#ifdef WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif
#ifdef WIN32
int gettimeofday(struct timeval *tp, void *tzp)
{
  time_t clock;
  struct tm tm;
  SYSTEMTIME wtm;
  GetLocalTime(&wtm);
  tm.tm_year   = wtm.wYear - 1900;
  tm.tm_mon   = wtm.wMonth - 1;
  tm.tm_mday   = wtm.wDay;
  tm.tm_hour   = wtm.wHour;
  tm.tm_min   = wtm.wMinute;
  tm.tm_sec   = wtm.wSecond;
  tm. tm_isdst  = -1;
  clock = mktime(&tm);
  tp->tv_sec = clock;
  tp->tv_usec = wtm.wMilliseconds * 1000;
  return (0);
}
#endif

typedef struct
{
  uint32_t length;
  uint32_t length_received;
  uint32_t length_total;

  uint16_t count;
  uint16_t count_total;

} cmd_flag;

cmd_flag flag;

static FILE      *opencr_fp;
static uint32_t   opencr_fpsize;


ser_handler stm32_ser_id = ( ser_handler )-1;


#define GET_CALC_TIME(x)	( (int)(x / 1000) + ((float)(x % 1000))/1000 )

#define FLASH_TX_BLOCK_LENGTH	(20*1024)
#define FLASH_RX_BLOCK_LENGTH	(128)
#define FLASH_PACKET_LENGTH   	200

#define DOWNLOAD_ID 0
#define JUMP_ID 1
#define REBOOT_ID 2

uint32_t tx_buf[256*1024/4];
uint32_t rx_buf[256*1024/4];

char err_msg_str[512];


int opencr_ld_down( int argc, const char **argv );
int opencr_ld_jump_to_boot( char *portname );
int opencr_ld_flash_write( uint32_t addr, uint8_t msg_id, uint8_t *p_data, uint32_t length, uint8_t need_reply );
BOOL opencr_ld_flash_read( uint32_t addr, uint8_t *p_data );
int opencr_ld_flash_erase( uint32_t length  );

uint32_t opencr_ld_file_read_data( uint8_t *dst, uint32_t len );
err_code_t send_cmd_and_rece_rpl(uint8_t msg_id, uint32_t addr, void *command_buf, uint32_t command_len, void *reply_buf, uint8_t need_reply);
void opencr_ld_write_err_msg( const char *fmt, ...);
void opencr_ld_print_err_msg(void);
err_code_t download_firmware();
err_code_t cmd_reboot();
static long iclock();
int read_byte( void );
int write_bytes( char *p_data, int len );
void delay_ms( int WaitTime );
uint32_t crc_calc( uint32_t crc_in, uint8_t data_in );

err_code_t cmd_flash_fw_write_packet(uint8_t msg_id, uint16_t addr, uint8_t *p_data, uint8_t length, uint8_t is_last_packet, uint8_t need_reply, uint32_t flash_offset );

err_code_t cmd_jump_to_fw(void);
void resp_ack(uint8_t ch, mavlink_ack_t *p_ack);
BOOL cmd_flash_fw_read_packet(uint8_t *rx_buf);




/*---------------------------------------------------------------------------
     TITLE   : opencr_ld_main
     WORK    :
---------------------------------------------------------------------------*/
int opencr_ld_main( int argc, const char **argv )
{
  long baud;
  baud = strtol( argv[ 2 ], NULL, 10 );

  printf("opencr_ld_main \r\n");

  opencr_ld_down( argc, argv );

  return 0;
}


/*---------------------------------------------------------------------------
     TITLE   : opencr_ld_down
     WORK    :
---------------------------------------------------------------------------*/
int opencr_ld_down( int argc, const char **argv )
{
  int i;
  int j;
  int ret = 0;
  err_code_t err_code = OK;
  long t, dt;
  float calc_time;
  uint32_t fw_size = 256*1024;
  uint8_t  board_str[16];
  uint8_t  board_str_len;
  uint32_t board_version;
  uint32_t board_revision;
  uint32_t crc;
  uint32_t crc_ret = 0;
  uint8_t  *p_buf_crc;
  char *portname;
  uint32_t baud;
  uint8_t  block_buf[FLASH_TX_BLOCK_LENGTH];
  uint32_t addr;
  uint32_t len;
  uint8_t jump_to_fw = 0;
  uint8_t retry;


  baud     = strtol( argv[ 2 ], NULL, 10 );
  portname = (char *)argv[ 1 ];

  if( argc >= 5 && strlen(argv[ 4 ])==1 && strncmp(argv[ 4 ], "1", 1)==0 )
  {
    jump_to_fw = 1;
  }

  if( ( opencr_fp = fopen( argv[ 3 ], "rb" ) ) == NULL )
  {
    fprintf( stderr, "Unable to open %s\n", argv[ 3 ] );
    exit( 1 );
  }
  else
  {
    fseek( opencr_fp, 0, SEEK_END );
    opencr_fpsize = ftell( opencr_fp );
    fseek( opencr_fp, 0, SEEK_SET );
    printf(">>\r\n");
    printf("file name : %s \r\n", argv[3]);
    printf("file size : %d KB\r\n", opencr_fpsize/1024);
  }

  fw_size = opencr_fpsize;

  // Open port
  if( ( stm32_ser_id = ser_open( portname ) ) == ( ser_handler )-1 )
  {
    printf("Fail to open port 1\n");
    return -1;
  }
  else
  {
    printf("Open port OK\n");
  }

  // Setup port
  ser_setupEx( stm32_ser_id, 115200, SER_DATABITS_8, SER_PARITY_NONE, SER_STOPBITS_1, 1 );

  printf("Clear Buffer Start\n");
  ser_set_timeout_ms( stm32_ser_id, SER_NO_TIMEOUT );
  while( read_byte() != -1 );
  ser_set_timeout_ms( stm32_ser_id, 1000 );
  printf("Clear Buffer End\n");

  t = iclock();
  download_firmware();
  dt = iclock() - t;

  printf("flash_write : %d : %f sec \r\n", ret,  GET_CALC_TIME(dt));

  //cmd_reboot();
  cmd_jump_to_fw();

  if( ret < 0 )
  {
    ser_close( stm32_ser_id );
    fclose( opencr_fp );
    opencr_ld_print_err_msg();
    printf("[FAIL] Download \r\n");
    return -2;
  }
  /*
  memset(rx_buf, 0, 768*1024);
  t = iclock();
  ret = opencr_ld_flash_read( 0, (uint8_t *)rx_buf, fw_size );
  dt = iclock() - t;
  printf("opencr_ld_flash_read : %d : %f sec \r\n", ret,  GET_CALC_TIME(dt));

  ret = 0;
  for( i=0; i<fw_size/4; i++ )
  {
    if( tx_buf[i] != rx_buf[i] )
    {
      printf("Compare Error : 0x%X \r\n", i*4);
      ret = -1;
      break;
    }
  }
  if( ret == 0 )
  {
    printf("Compare OK \r\n");
  }
  */


/*
  for(i = 0; i < 3; i++)
  {
    if( i > 0)
    {
      printf("CRC Retry : %d\r\n", i);
    }

    t = iclock();
    err_code = cmd_flash_fw_verify( fw_size, crc, &crc_ret );
    dt = iclock() - t;
    
    if(err_code == OK)
    {
      break;
    }
  }
  */

  if( err_code == OK )
  {
    printf("CRC OK %X %X %f sec\r\n", crc, crc_ret, GET_CALC_TIME(dt));
  }
  else
  {
    printf("CRC Fail : 0x%X : %X, %X %f sec\r\n", err_code, crc, crc_ret, GET_CALC_TIME(dt));
    printf("[FAIL] Download \r\n");
    ser_close( stm32_ser_id );
    fclose( opencr_fp );
    return -3;
  }

  printf("[OK] Download \r\n");
/*
  if( jump_to_fw == 1 )
  {
    printf("jump_to_fw \r\n");
    cmd_jump_to_fw();
  }
*/
  ser_close( stm32_ser_id );
  fclose( opencr_fp );


  for (int i=0; i<6; i++)
  {
    delay_ms(500);
    if (ser_port_is_ready(portname) > 0)
    {
      printf("jump finished\r\n");
      delay_ms(100);
      break;
    }
  }

  return ret;
}


/*---------------------------------------------------------------------------
     TITLE   : opencr_ld_file_read_data
     WORK    :
---------------------------------------------------------------------------*/
int opencr_ld_jump_to_boot( char *portname )
{
  bool ret;


  // Open port
  if( ( stm32_ser_id = ser_open( portname ) ) == ( ser_handler )-1 )
  {
    printf("Fail to open port 1 : %s\n", portname);
    return -1;
  }

  // Setup port
  ser_setupEx( stm32_ser_id, 1200, SER_DATABITS_8, SER_PARITY_NONE, SER_STOPBITS_1, 1 );

  write_bytes("OpenCR 5555AAAA", 15);
  ser_close( stm32_ser_id );

  delay_ms(1500);

  return 0;
}


/*---------------------------------------------------------------------------
     TITLE   : opencr_ld_file_read_data
     WORK    :
---------------------------------------------------------------------------*/
uint32_t opencr_ld_file_read_data( uint8_t *dst, uint32_t len )
{
  size_t readbytes = 0;

  if( !feof( opencr_fp ) )
  {
    readbytes = fread( dst, 1, len, opencr_fp );
  }
  return ( uint32_t )readbytes;
}


/*---------------------------------------------------------------------------
     TITLE   : opencr_ld_flash_write
     WORK    :
---------------------------------------------------------------------------*/
int opencr_ld_flash_write( uint32_t addr, uint8_t msg_id, uint8_t *p_data, uint32_t length, uint8_t need_reply )
{
  int ret = 0;
  err_code_t err_code = OK;
  uint32_t block_length;
  uint16_t packet_cnt;
  uint32_t written_packet_length;
  uint32_t written_total_length;
  uint32_t packet_length = 200;
  uint32_t i = 0;
  uint8_t is_last_packet;

  if( length > FLASH_TX_BLOCK_LENGTH )
  {
      printf("length too large %d\r\n");
      return -1;
  }

  packet_cnt = length/FLASH_PACKET_LENGTH;
  if( length%FLASH_PACKET_LENGTH > 0 )
  {
      packet_cnt += 1;
  }

  printf("packet_cnt = %d\r\n",packet_cnt);
  written_packet_length = 0;
  //for( i=0; i<packet_cnt; i++ )
  do
  {
      //printf("packet_cnt = %d\r\n",i);
      packet_length = length - written_packet_length;
      if( packet_length > FLASH_PACKET_LENGTH )
      {
          packet_length = FLASH_PACKET_LENGTH;
      }

      if( i == (packet_cnt - 1) )
      {
          is_last_packet = 1;
      }
      else
      {
          is_last_packet = 0;
      }
      printf("is_last_packet = %d\r\n",is_last_packet);
      err_code = cmd_flash_fw_write_packet(msg_id, written_packet_length, &p_data[written_packet_length], packet_length, is_last_packet, need_reply, addr);
      if( err_code != OK )
      {
          opencr_ld_write_err_msg("cmd_flash_fw_send_block ERR : 0x%04X\r\n", err_code);
          return -2;
      }

      written_packet_length += packet_length;
      i++;
  }while(is_last_packet != 1);

  //printf("%d : %d, %d, %d \r\n", written_packet_length, block_length, block_cnt, packet_length);
  return ret;
}
/*---------------------------------------------------------------------------
     TITLE   : opencr_ld_flash_read
     WORK    :
---------------------------------------------------------------------------*/
BOOL opencr_ld_flash_read( uint32_t addr, uint8_t *p_data )
{
    err_code_t err_code = OK;
    BOOL ret = FALSE;
    mavlink_message_t p_msg;
    mavlink_ack_t mav_ack;
    mavlink_packet_t mav_data;

    do
    {
        if (msg_get_resp(0, &p_msg, 500) == TRUE)
        {
            mavlink_msg_packet_decode(&p_msg, &mav_data);
            printf("is_last_packet %d \r\n", mav_data.is_last_packet);

            if (mav_data.addr == 0)
            {
                flag.count = 0;
                flag.count_total = 0;

                flag.length = 0;
                flag.length_total = 0;
                flag.length_received = 0;

                memset(p_data, 0, FLASH_TX_BLOCK_LENGTH);
            }

            if ((flag.length_received + mav_data.length) <= FLASH_TX_BLOCK_LENGTH)
            {
                memcpy(&p_data[flag.length_received], &mav_data.data[0], mav_data.length);
            }

            flag.count += 1;
            flag.length_received += mav_data.length;

            flag.count_total += 1;
            flag.length_total += mav_data.length;

            mav_ack.msg_id = p_msg.msgid;
            mav_ack.err_code = err_code;
            resp_ack(0, &mav_ack);

            if (mav_data.is_last_packet == 1)
            {
                ret = TRUE;
                break;
            }
        }
        else {
            printf("error");
        }
    } while (1);

    return ret;
}


/*---------------------------------------------------------------------------
     TITLE   : opencr_ld_flash_erase
     WORK    :
---------------------------------------------------------------------------*/
static long iclock()
{
	struct timeval tv;
	gettimeofday (&tv, NULL);
	return (tv.tv_sec * 1000 + tv.tv_usec / 1000);
}


/*---------------------------------------------------------------------------
     TITLE   : delay_ms
     WORK    :
---------------------------------------------------------------------------*/
void delay_ms( int WaitTime )
{
  int i;

  #ifdef WIN32_BUILD
  Sleep(WaitTime);
  #else
  for( i=0; i<WaitTime; i++ )
  {
    usleep(1000);
  }
  #endif
}


/*---------------------------------------------------------------------------
     TITLE   : read_byte
     WORK    :
---------------------------------------------------------------------------*/
int read_byte( void )
{
  return ser_read_byte( stm32_ser_id );
}



/*---------------------------------------------------------------------------
     TITLE   : read_bytes
     WORK    :
---------------------------------------------------------------------------*/
int read_bytes( uint8_t *pData, uint32_t size )
{
  return read( stm32_ser_id, pData, size ); //ser_read( stm32_ser_id, pData, size );
  //return ser_read( stm32_ser_id, pData, size );
}



/*---------------------------------------------------------------------------
     TITLE   : write_bytes
     WORK    :
---------------------------------------------------------------------------*/
int write_bytes( char *p_data, int len )
{
  int written_len;

  written_len = ser_write( stm32_ser_id, (const u8 *)p_data, len );

  return written_len;
}


/*---------------------------------------------------------------------------
     TITLE   : cmd_read_version
     WORK    :
---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
     TITLE   : cmd_read_board_name
     WORK    :
---------------------------------------------------------------------------*/



/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_erase
     WORK    :
---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_write_begin
     WORK    :
---------------------------------------------------------------------------*/



/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_write_end
     WORK    :
---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_write_packet
     WORK    :
---------------------------------------------------------------------------*/
err_code_t cmd_flash_fw_write_packet(uint8_t msg_id, uint16_t addr, uint8_t *p_data, uint8_t length, uint8_t is_last_packet, uint8_t need_reply, uint32_t flash_offset )
{
  err_code_t err_code = OK;
  mavlink_message_t tx_msg;
  mavlink_message_t rx_msg;
  mavlink_ack_t     ack_msg;
  uint8_t resp = need_reply;

  mavlink_msg_packet_pack(0, 0, msg_id, &tx_msg, resp, addr, length, flash_offset, 0, is_last_packet, p_data);
  msg_send(0, &tx_msg);



  if( msg_get_resp(0, &rx_msg, 500) == TRUE )
  {
      mavlink_msg_ack_decode( &rx_msg, &ack_msg);

      if( tx_msg.msgid == ack_msg.msg_id ) err_code = ack_msg.err_code;
      else                                 err_code = ERR_MISMATCH_ID;
  }
  else
  {
      err_code = ERR_TIMEOUT;
  }

  return err_code;
}

void resp_ack(uint8_t ch, mavlink_ack_t *p_ack)
{
  mavlink_message_t mav_msg;

  mavlink_msg_ack_pack_chan(0, 0, ch, &mav_msg, p_ack->msg_id, p_ack->err_code, p_ack->length_received, p_ack->ack_length, p_ack->data);

  msg_send(ch, &mav_msg);
}

BOOL cmd_flash_fw_read_packet(uint8_t *rx_buf)
{
  err_code_t err_code = OK;
  BOOL ret = FALSE;
  mavlink_message_t p_msg;
  mavlink_ack_t mav_ack;
  mavlink_packet_t mav_data;

  do
  {
    if (msg_get_resp(0, &p_msg, 50) == TRUE)
    {
      mavlink_msg_packet_decode(&p_msg, &mav_data);
      printf("is_last_packet %d \r\n", mav_data.is_last_packet);

      if (mav_data.addr == 0)
      {
        flag.count = 0;
        flag.count_total = 0;

        flag.length = 0;
        flag.length_total = 0;
        flag.length_received = 0;

        memset(rx_buf, 0, FLASH_TX_BLOCK_LENGTH);
      }

      if ((flag.length_received + mav_data.length) <= FLASH_TX_BLOCK_LENGTH)
      {
        memcpy(&rx_buf[flag.length_received], &mav_data.data[0], mav_data.length);
      }

      flag.count += 1;
      flag.length_received += mav_data.length;

      flag.count_total += 1;
      flag.length_total += mav_data.length;


      mav_ack.msg_id = p_msg.msgid;
      mav_ack.err_code = err_code;
      resp_ack(0, &mav_ack);

      if (mav_data.is_last_packet == 1)
      {
        ret = TRUE;
        break;
      }
    }
    else {
        printf("error");
    }
  } while (1);

  return ret;
}
/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_send_block_multi
     WORK    :
---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_write_block
     WORK    :
---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_read_block
     WORK    :
---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
     TITLE   : cmd_jump_to_fw
     WORK    :
---------------------------------------------------------------------------*/
/*
err_code_t cmd_jump_to_fw(void)
{
  err_code_t err_code = OK;
  mavlink_message_t tx_msg;
  mavlink_message_t rx_msg;
  mavlink_ack_t     ack_msg;
  uint8_t param[8];
  uint8_t resp = 0;


  mavlink_msg_jump_to_fw_pack(0, 0, &tx_msg, resp, param);
  msg_send(0, &tx_msg);


  if( resp == 1 )
  {
    if( msg_get_resp(0, &rx_msg, 500) == TRUE )
    {
      mavlink_msg_ack_decode( &rx_msg, &ack_msg);

      if( tx_msg.msgid == ack_msg.msg_id ) err_code = ack_msg.err_code;
      else                                 err_code = ERR_MISMATCH_ID;
    }
    else
    {
      err_code = ERR_TIMEOUT;
    }
  }
  return err_code;
}
*/

/*---------------------------------------------------------------------------
     TITLE   : crc_calc
     WORK    :
---------------------------------------------------------------------------*/
uint32_t crc_calc( uint32_t crc_in, uint8_t data_in )
{

  crc_in  ^= data_in;
  crc_in  += data_in;

  return crc_in;
}


/*---------------------------------------------------------------------------
     TITLE   : opencr_ld_write_err_msg
     WORK    :
---------------------------------------------------------------------------*/
void opencr_ld_write_err_msg( const char *fmt, ...)
{
  int32_t ret = 0;
  va_list arg;
  va_start (arg, fmt);
  int32_t len;

  len = vsnprintf(err_msg_str, 255, fmt, arg);
  va_end (arg);
}


/*---------------------------------------------------------------------------
     TITLE   : opencr_ld_write_err_msg
     WORK    :
---------------------------------------------------------------------------*/
void opencr_ld_print_err_msg(void)
{
  uint32_t len;

  len = strlen(err_msg_str);

  if( len > 0 && len < 500 )
  {
    printf("%s", err_msg_str);
  }
}


err_code_t send_cmd_and_rece_rpl(uint8_t msg_id, uint32_t addr, void *command_buf, uint32_t command_len, void *reply_buf, uint8_t need_reply)
{
    err_code_t ret = OK;

    ret = opencr_ld_flash_write(addr, msg_id, (uint8_t *)command_buf, command_len, need_reply );

    if((ret == OK)&&(need_reply == 1))
    {
        opencr_ld_flash_read( 0, (uint8_t *)reply_buf);
    }

    return ret;
}

err_code_t download_firmware()
{
    err_code_t ret = OK;
    uint32_t addr;
    uint32_t len;
    uint8_t need_reply = 1;
    addr = 0;
    while(1)
    {
      len = opencr_ld_file_read_data( tx_buf, FLASH_TX_BLOCK_LENGTH);
      if( len == 0 ) break;

      printf("len : %d\r\n", len);

      ret = send_cmd_and_rece_rpl(DOWNLOAD_ID, addr, tx_buf, len, rx_buf, need_reply);
      addr += len;
    }
    return ret;
}

err_code_t cmd_jump_to_fw()
{
    err_code_t ret = OK;
    uint32_t addr = 0;
    uint32_t len = 1;
    uint8_t need_reply = 0;

    ret = send_cmd_and_rece_rpl(JUMP_ID, addr, tx_buf, len, rx_buf, need_reply);

    return ret;
}

err_code_t cmd_reboot()
{
    err_code_t ret = OK;
    uint32_t addr = 0;
    uint32_t len = 1;
    uint8_t need_reply = 0;

    ret = send_cmd_and_rece_rpl(REBOOT_ID, addr, tx_buf, len, rx_buf, need_reply);

    return ret;
}
