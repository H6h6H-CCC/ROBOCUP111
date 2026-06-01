#include "stm32f4xx.h"                  // Device header
#include "main.h"
//#include duoji

extern uint8_t stat;
extern uint8_t QRPacke[2];
extern uint8_t QRPacke1[2];
extern uint8_t Coulor[5];
uint8_t outLetters[5];
uint8_t outLetter[4];
int QR;
int ball_1[5] = {0};  
int ball_2[5]={0};
void BuildBall1OrderFromQr(void);

/**
 * ������ASCII�ַ�����Ϊ����������ǰ�����ֽ����x���������ֽ����y��
 * @param bytes ����Ϊ6���ֽ����飬ÿ���ֽ�ӦΪ'0'~'9'��ASCII��
 * @param x ָ��洢��һ�������ı�������Χ0~999��
 * @param y ָ��洢�ڶ��������ı�������Χ0~999��
 */
void decode_six_to_xy(const uint8_t bytes[6], int *x, int *y) {
    *x = (bytes[0] - '0') * 100 + (bytes[1] - '0') * 10 + (bytes[2] - '0');
    *y = (bytes[3] - '0') * 100 + (bytes[4] - '0') * 10 + (bytes[5] - '0');
}

//    0   1    2    3    4
//    a   b    c    d    e

void GetTask1SequenceHexArray(void)
{
	int tens = QRPacke[0] - '0';   // ʮλ����
    int units = QRPacke[1] - '0';  // ��λ����
    QR= tens * 10 + units;
	QR=1;
	if(stat==1)
	{
		switch(QR)
		{
			case 1:  outLetters[0]='k'; outLetters[1]='w'; outLetters[2]='r'; outLetters[3]='g'; outLetters[4]='b'; break;
			case 2:  outLetters[0]='w'; outLetters[1]='k'; outLetters[2]='r'; outLetters[3]='g'; outLetters[4]='b'; break;
			case 3:  outLetters[0]='w'; outLetters[1]='k'; outLetters[2]='g'; outLetters[3]='r'; outLetters[4]='b'; break;
			case 4:  outLetters[0]='b'; outLetters[1]='w'; outLetters[2]='k'; outLetters[3]='r'; outLetters[4]='g'; break;
			case 5:  outLetters[0]='w'; outLetters[1]='r'; outLetters[2]='b'; outLetters[3]='k'; outLetters[4]='g'; break;
			case 6:  outLetters[0]='k'; outLetters[1]='r'; outLetters[2]='b'; outLetters[3]='w'; outLetters[4]='g'; break;
			case 7:  outLetters[0]='b'; outLetters[1]='g'; outLetters[2]='k'; outLetters[3]='w'; outLetters[4]='r'; break;
			case 8:  outLetters[0]='g'; outLetters[1]='w'; outLetters[2]='b'; outLetters[3]='k'; outLetters[4]='r'; break;
			case 9:  outLetters[0]='w'; outLetters[1]='g'; outLetters[2]='k'; outLetters[3]='b'; outLetters[4]='r'; break;
			case 10: outLetters[0]='k'; outLetters[1]='r'; outLetters[2]='b'; outLetters[3]='g'; outLetters[4]='w'; break;
			case 11: outLetters[0]='r'; outLetters[1]='b'; outLetters[2]='g'; outLetters[3]='k'; outLetters[4]='w'; break;
			case 12: outLetters[0]='g'; outLetters[1]='r'; outLetters[2]='k'; outLetters[3]='b'; outLetters[4]='w'; break;
			case 13: outLetters[0]='w'; outLetters[1]='r'; outLetters[2]='b'; outLetters[3]='g'; outLetters[4]='k'; break;
			case 14: outLetters[0]='r'; outLetters[1]='g'; outLetters[2]='w'; outLetters[3]='b'; outLetters[4]='k'; break;
			case 15: outLetters[0]='b'; outLetters[1]='w'; outLetters[2]='g'; outLetters[3]='r'; outLetters[4]='k'; break;
			case 16: outLetters[0]='g'; outLetters[1]='b'; outLetters[2]='r'; outLetters[3]='w'; outLetters[4]='k'; break;
			default: 
				outLetters[0]=outLetters[1]=outLetters[2]=outLetters[3]=outLetters[4]=0;
				break;
		}
		BuildBall1OrderFromQr();
	}//c  b  a
	else if(stat==3)
	{
		int tens = QRPacke1[0] - '0';   // ʮλ
    	int units = QRPacke1[1] - '0';  // λ
    	QR= tens * 10 + units;
		QR=2;
		switch(QR)
		{
			case 1:  outLetter[0]='a'; outLetter[1]='b'; outLetter[2]='c';
			break;
			case 2:  outLetter[0]='a'; outLetter[1]='c'; outLetter[2]='b';
			break;
			case 3:  outLetter[0]='b'; outLetter[1]='a'; outLetter[2]='c';
			break;
			case 4:  outLetter[0]='b'; outLetter[1]='c'; outLetter[2]='a';
			break;
			case 5:  outLetter[0]='c'; outLetter[1]='a'; outLetter[2]='b';
			break;
			case 6:  outLetter[0]='c'; outLetter[1]='b'; outLetter[2]='a';
			break;
			default: 
				outLetter[0]=outLetter[1]=outLetter[2]=outLetter[3]=0;
				break;
		}		
	}
}


uint8_t panduan(uint8_t i)
{
	uint8_t j=0;
	for(j=0;j<5;j++)
	{
		if(Coulor[j]==outLetters[i])
		{
			return j;
		}
	}
	return 0xFF;
}

void BuildBall1OrderFromQr(void)
{
	uint8_t i = 0;
	uint8_t pos = 0;
	for(i = 0; i < 5; i++)
	{
		pos = panduan(i);
		if(pos < 5)
		{
			ball_1[i] = (int)(pos + 1);
		}
		else
		{
			ball_1[i] = 0;
		}
	}
}

void BuildBall2OrderFromQr(void)
{
    // 固定出球顺序
    const uint8_t fixedOrder[3] = {'b', 'a', 'c'};
    uint8_t i, j;

    // 先清空 ball_2
    for (i = 0; i < 5; i++) {
        ball_2[i] = 0;
    }

    // 对每个出球位置 i，找到对应颜色在 outLetter 中的位置（1-based）
    for (i = 0; i < 3; i++) {
        uint8_t targetColor = fixedOrder[i];
        for (j = 0; j < 3; j++) {
            if (outLetter[j] == targetColor) {
                ball_2[i] = j + 1;   // 仓号 = 索引 + 1
                break;
            }
        }
        // 理论上一定找得到，若未找到 ball_2[i] 保持 0（安全）
    }
}













