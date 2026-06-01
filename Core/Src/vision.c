//#include "vision.h"
//#include "string.h"

////全局变量定义与初始化
//uint8_t usart2_rx_buf[4];
//uint8_t usart3_rx_buf[10];
//uint8_t QRPacket[2];
//uint8_t ColorPacket[5];
//uint8_t LetterPacket[3];
//uint8_t LocatePacket[6];
//uint8_t qr_flag;
//uint8_t color_flag;
//uint8_t letter_flag;
//uint8_t locate_flag;

////全局句柄
//extern UART_HandleTypeDef huart2;
//extern UART_HandleTypeDef huart3;
//extern DMA_HandleTypeDef hdma_usart2_rx;
//extern DMA_HandleTypeDef hdma_usart3_rx;

////void MX_USART2_UART_Init(void)
////{
////  huart2.Instance = USART2;
////  huart2.Init.BaudRate = 115200;
////  huart2.Init.WordLength = UART_WORDLENGTH_8B;
////  huart2.Init.StopBits = UART_STOPBITS_1;
////  huart2.Init.Parity = UART_PARITY_NONE;
////  huart2.Init.Mode = UART_MODE_TX_RX;
////  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
////  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
////  if (HAL_UART_Init(&huart2) != HAL_OK)
////  {
////    Error_Handler();
////  }
////  //开启USART2DMA单次接收，固定4字节
////  HAL_UART_Receive_DMA(&huart2, usart2_rx_buf, 4);
////}

////void MX_USART3_UART_Init(void)
////{
////  huart3.Instance = USART3;
////  huart3.Init.BaudRate = 115200;
////  huart3.Init.WordLength = UART_WORDLENGTH_8B;
////  huart3.Init.StopBits = UART_STOPBITS_1;
////  huart3.Init.Parity = UART_PARITY_NONE;
////  huart3.Init.Mode = UART_MODE_TX_RX;
////  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
////  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
////  if (HAL_UART_Init(&huart3) != HAL_OK)
////  {
////    Error_Handler();
////  }
////  //开启USART3DMA单次接收，最大10字节
////  HAL_UART_Receive_DMA(&huart3, usart3_rx_buf, 10);
////}

////重写HAL库弱定义回调，DMA接收完成后触发回调，解析数据并重开接收



////视觉数据包解析函数，判断串口、包头包尾有效性，提取中间有效数据到对应变量，置位标志位
//void Vision_Parse_Packet(void)
//{
//  //USART2二维码包解析：包头0xAA + 2字符 + 包尾0x55
//  if(usart2_rx_buf[0] == 0xAA && usart2_rx_buf[3] == 0x55)
//  {
//    QRPacket[0] = usart2_rx_buf[1];
//    QRPacket[1] = usart2_rx_buf[2];
//    qr_flag = 1; //置位解析完成标志
//  }
//  else
//  {
//    qr_flag = 0; //包头包尾无效，标志位清0
//  }

//  //USART3视觉包解析，判断不同包头，匹配对应包尾和长度
//  //1.物料颜色顺序：包头0xAB + 5位颜色首字母（特殊黑色对应k）+ 包尾0x55
//  if(usart3_rx_buf[0] == 0xAB && usart3_rx_buf[6] == 0x55)
//  {
//    memcpy(ColorPacket, &usart3_rx_buf[1], 5); //拷贝5位有效字符
//    color_flag = 1;
//    letter_flag = 0;
//    locate_flag = 0;
//  }
//  //2.字母摆放顺序：包头0xAC + 3位字母顺序 + 0x55
//  else if(usart3_rx_buf[0] == 0xAC && usart3_rx_buf[4] == 0x55)
//  {
//    memcpy(LetterPacket, &usart3_rx_buf[1], 3); //拷贝3位有效字符
//    letter_flag = 1;
//    color_flag = 0;
//    locate_flag = 0;
//  }
//  //3.定位数据：0xAD + 6位坐标数据 + 0x55
//  else if(usart3_rx_buf[0] == 0xAD && usart3_rx_buf[7] == 0x55)
//  {
//    memcpy(LocatePacket, &usart3_rx_buf[1], 6); //拷贝6位有效字符
//    locate_flag = 1;
//    color_flag = 0;
//    letter_flag = 0;
//  }
//  //包无效，所有标志位清0
//  else
//  {
//    color_flag = 0;
//    letter_flag = 0;
//    locate_flag = 0;
//  }
//}
