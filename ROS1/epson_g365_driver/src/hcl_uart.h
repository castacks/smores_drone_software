//==============================================================================
//
//	hcl_uart.h - Seiko Epson Hardware Control Library
//
//
//
//  THE SOFTWARE IS RELEASED INTO THE PUBLIC DOMAIN.
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, NONINFRINGEMENT, 
//  SECURITY, SATISFACTORY QUALITY, AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT
//  SHALL EPSON BE LIABLE FOR ANY LOSS, DAMAGE OR CLAIM, ARISING FROM OR IN CONNECTION
//  WITH THE SOFTWARE OR THE USE OF THE SOFTWARE.
//
//==============================================================================
#ifndef HCL_UART_H_
#define HCL_UART_H_

// Prototypes for generic UART functions
int uartInit(const char* comPortPath, int baudrate);
int uartRelease(int comPort);
int readComPort(int comPort, unsigned char* bytesToRead, int size);
int writeComPort(int comPort, unsigned char* bytesToWrite, int size);
int numBytesReadComPort(int comPort);
int purgeComPort(int comPort);

#endif /* HCL_UART_H_ */
