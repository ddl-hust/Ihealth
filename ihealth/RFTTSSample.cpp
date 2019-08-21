#include "RFTTSSample.h"
#include <stdlib.h>
#include <stdio.h>
#include <windows.h>
#include <conio.h>
#include <errno.h>
#include "qtts.h"
#include "msp_cmn.h"
#include "msp_errors.h"

/* wav��Ƶͷ����ʽ */
typedef struct _wave_pcm_hdr
{
    char riff[4]; // = "RIFF"
    int size_8; // = FileSize - 8
    char wave[4]; // = "WAVE"
    char fmt[4]; // = "fmt "
    int fmt_size; // = ��һ���ṹ��Ĵ�С : 16

    short int format_tag; // = PCM : 1
    short int channels; // = ͨ���� : 1
    int samples_per_sec; // = ������ : 8000 | 6000 | 11025 | 16000
    int avg_bytes_per_sec; // = ÿ���ֽ��� : samples_per_sec * bits_per_sample / 8
    short int block_align; // = ÿ�������ֽ��� : wBitsPerSample / 8
    short int bits_per_sample; // = ����������: 8 | 16

    char data[4]; // = "data";
    int data_size; // = �����ݳ��� : FileSize - 44
} wave_pcm_hdr;

/* Ĭ��wav��Ƶͷ������ */
wave_pcm_hdr default_wav_hdr = {{'R', 'I', 'F', 'F'}, 0, {'W', 'A', 'V', 'E'}, {'f', 'm', 't', ' '}, 16, 1, 1, 16000, 32000, 2, 16, {'d', 'a', 't', 'a'}, 0};


/* �ı��ϳ� */
int text_to_speech(const char *src_text, const char *des_path, const char *params)
{
    int ret = -1;
    FILE *fp = NULL;
    const char *sessionID = NULL;
    unsigned int audio_len = 0;
    wave_pcm_hdr wav_hdr = default_wav_hdr;
    int synth_status = MSP_TTS_FLAG_STILL_HAVE_DATA;

    if (NULL == src_text || NULL == des_path) {
        return ret;
    }
    fp = fopen(des_path, "wb");
    if (NULL == fp) {
        return ret;
    }
    /* ��ʼ�ϳ� */
    sessionID = QTTSSessionBegin(params, &ret);
    if (MSP_SUCCESS != ret) {
        fclose(fp);
        return ret;
    }
    ret = QTTSTextPut(sessionID, src_text, (unsigned int)strlen(src_text), NULL);
    if (MSP_SUCCESS != ret) {
        QTTSSessionEnd(sessionID, "TextPutError");
        fclose(fp);
        return ret;
    }
    fwrite(&wav_hdr, sizeof(wav_hdr), 1, fp); //����wav��Ƶͷ��ʹ�ò�����Ϊ16000
    while (1) {
        /* ��ȡ�ϳ���Ƶ */
        const void *data = QTTSAudioGet(sessionID, &audio_len, &synth_status, &ret);
        if (MSP_SUCCESS != ret) break;
        if (NULL != data) {
            fwrite(data, audio_len, 1, fp);
            wav_hdr.data_size += audio_len; //����data_size��С
        }
        if (MSP_TTS_FLAG_DATA_END == synth_status) break;
        Sleep(150); //��ֹƵ��ռ��CPU
    } //�ϳ�״̬synth_statusȡֵ����ġ�Ѷ��������API�ĵ���
    if (MSP_SUCCESS != ret) {
        QTTSSessionEnd(sessionID, "AudioGetError");
        fclose(fp);
        return ret;
    }
    /* ����wav�ļ�ͷ���ݵĴ�С */
    wav_hdr.size_8 += wav_hdr.data_size + (sizeof(wav_hdr) - 8);

    /* ��������������д���ļ�ͷ��,��Ƶ�ļ�Ϊwav��ʽ */
    fseek(fp, 4, 0);
    fwrite(&wav_hdr.size_8, sizeof(wav_hdr.size_8), 1, fp); //д��size_8��ֵ
    fseek(fp, 40, 0); //���ļ�ָ��ƫ�Ƶ��洢data_sizeֵ��λ��
    fwrite(&wav_hdr.data_size, sizeof(wav_hdr.data_size), 1, fp); //д��data_size��ֵ
    fclose(fp);
    fp = NULL;
    /* �ϳ���� */
    ret = QTTSSessionEnd(sessionID, "Normal");
    return ret;
}


int RFTTSSample(const char *text, const char *filename)
{
    int ret = MSP_SUCCESS;
    const char *login_params = "appid = 585c997a, work_dir = ."; //��¼����,appid��msc���,��������Ķ�

    const char *session_begin_params = "voice_name = xiaoyan, text_encoding = gb2312, sample_rate = 16000, speed = 50, volume = 50, pitch = 50, rdn = 2";
    ret = MSPLogin(NULL, NULL, login_params);
    if (MSP_SUCCESS != ret) {
        goto exit;
    }

    ret = text_to_speech(text, filename, session_begin_params);

exit:
    MSPLogout();
    return ret;
}