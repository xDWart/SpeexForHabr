#include "speexx.h"

//SPEEX variables
__IO uint16_t IN_Buffer[2][FRAME_SIZE];
__IO uint8_t Start_Encoding = 0;
uint8_t Index_Encoding = 0;
uint32_t Encoded_Frames = 0;

uint8_t REC_DATA[2][MAX_REC_FRAMES*ENCODED_FRAME_SIZE]; //сюда сохраняются закодированные данные
uint8_t* Rec_Data_ptr = &REC_DATA[0][0]; //указатель на кодируемые данные
uint8_t* Trm_Data_ptr; //указатель на передаваемые данные

int quality = 4, complexity=1, vbr=0, enh=1;/* SPEEX PARAMETERS, MUST REMAINED UNCHANGED */
SpeexBits bits; /* Holds bits so they can be read and written by the Speex routines */
void *enc_state, *dec_state;/* Holds the states of the encoder & the decoder */

void Speex_Init(void)
{
  /* Speex encoding initializations */ 
  speex_bits_init(&bits);
  enc_state = speex_encoder_init(&speex_nb_mode);
  speex_encoder_ctl(enc_state, SPEEX_SET_VBR, &vbr);
  speex_encoder_ctl(enc_state, SPEEX_SET_QUALITY,&quality);
  speex_encoder_ctl(enc_state, SPEEX_SET_COMPLEXITY, &complexity);
}

void EncodingVoice(void)
{
    uint8_t i;
    
    //====================Если одна из половинок буфера заполнена======================
    if(Start_Encoding > 0)
      { 
        Index_Encoding = Start_Encoding - 1;
        for (i=0;i<FRAME_SIZE;i++) IN_Buffer[Index_Encoding][i]^=0x8000;
        /* Flush all the bits in the struct so we can encode a new frame */
        speex_bits_reset(&bits);
        /* Encode the frame */
        speex_encode_int(enc_state, (spx_int16_t*)IN_Buffer[Index_Encoding], &bits);
        /* Copy the bits to an array of char that can be decoded */
        speex_bits_write(&bits, (char *)Rec_Data_ptr, ENCODED_FRAME_SIZE);
          
        Rec_Data_ptr += ENCODED_FRAME_SIZE;
        Encoded_Frames += 1;
        
        Start_Encoding = 0;	
      }
    
    if (Encoded_Frames == MAX_REC_FRAMES) {        
        __no_operation(); //первая половина данных готова, можно забирать из &REC_DATA[0][0]
    }
    
    if (Encoded_Frames == MAX_REC_FRAMES*2) {
        Rec_Data_ptr = &REC_DATA[0][0];
        Encoded_Frames = 0;
        __no_operation(); //вторая половина данных готова, можно забирать из &REC_DATA[1][0]
    }
}