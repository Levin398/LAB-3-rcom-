#include "linklayer.h"

//S, UN, I BYTES
#define SET_SIZE 5  // tamanho em bytes da trama SET
#define UA_SIZE 5  // tamanho em bytes da trama UA
#define DISC_SIZE 5 // tamanho em bytes da trama DISC


#define FLAG 0b01011100  // (0x5C) flag de inicio e fim

#define A_ER 0b00000001  // (0x01) Campo de Endereço (A) de commandos do Emissor, resposta do Receptor
#define A_RE 0b00000011  // (0x03) Campo de Endereço (A) de commandos do Receptor, resposta do Emissor

#define C_SET 0b00000011 // (0x03) Campo de Controlo - SET (set up)
#define C_DISC 0b00001011 // (0x0B) Campo de Controlo - DISC (disconnect)
#define C_UA 0b00000111 // (0x07) Campo de Controlo - UA (Unnumbered Acknowledgement)
#define C_RR(r) ((0b00000001) ^ (r) << (5)) // (0x01 OU 0x21) Campo de Controlo - RR (receiver ready / positive ACK))
#define C_REJ(r) ((0b00000101) ^ (r) << (5)) // (0x05 OU 0x25) Campo de Controlo - REJ (reject / negative ACK))
#define C_NS(r) ((0b00000000) ^ (r) << (1)) // (0x00 OU 0x02) Campo de Controlo - N(s) (Sequence number)
//  FLAG XOR 20 = 7c
//  ESC XOR 20 = 7d

#define BCC(a,c) (a ^ c)

#define ESC 0b01011101 // (0x5D) Escape octet


enum stateMachine {Start, FLAG_RCV, A_RCV, C_RCV, BCC_OK, DONE};

int failed = FALSE, attempt = 0;
int fd;
int s = 0;
int r = 1;

void atende(){
  printf("Alarme #%d\n", attempt);
  failed = TRUE;
  attempt++;
}

void determineState(enum stateMachine *state, unsigned char *checkBuffer, char byte, unsigned char *buf, int role){
  // TO-DO máquina de estados
  switch (*state)
  {
  case Start:
    if (byte == FLAG) *state = FLAG_RCV;    
    break;


  case FLAG_RCV:
    if (byte == A_ER) {
      checkBuffer[0] = byte;
      *state = A_RCV;
    }
    else if (byte!= FLAG){
      *state = Start;
    }
    break;


  case A_RCV:
    if (byte == C_UA && role == TRANSMITTER) {
      checkBuffer[1] = byte;
      *state = C_RCV;
    }
    else if (byte == C_SET && role == RECEIVER){
      checkBuffer[1] = byte;
      *state = C_RCV;    
      }
    else if (byte == FLAG){
      *state = FLAG_RCV;
    }
    else{
      *state = Start;
    }
    break;


  case C_RCV:
    if (byte == BCC(checkBuffer[0],checkBuffer[1])){
      *state = BCC_OK;
    }
    else if (byte == FLAG){
      *state = FLAG_RCV;
    }
    else{
      *state = Start;
    }
    break;


  case BCC_OK:
    if (byte == FLAG){
      *state = DONE;
    }
    else{
      *state = Start;
    }
    break;


  case DONE:
    break;
  }
}

void determineStateRR(enum stateMachine *state, unsigned char *checkBuffer, char byte, unsigned char *buf){
  switch (*state)
  {
  case Start:
    if (byte == FLAG) *state = FLAG_RCV;    
    break;


  case FLAG_RCV:
    if (byte == A_ER) {
      checkBuffer[0] = byte;
      *state = A_RCV;
    }
    else if (byte!= FLAG){
      *state = Start;
    }
    break;


  case A_RCV:
    if (byte == C_RR(r)) {
      checkBuffer[1] = byte;
      *state = C_RCV;
    }
    else if (byte == C_REJ(r)){
      checkBuffer[1] = byte;
      *state = C_RCV;    
      }
    else if (byte == FLAG){
      *state = FLAG_RCV;
    }
    else{
      *state = Start;
    }
    break;


  case C_RCV:
    if (byte == BCC(checkBuffer[0],checkBuffer[1])){
      *state = BCC_OK;
    }
    else if (byte == FLAG){
      *state = FLAG_RCV;
    }
    else{
      *state = Start;
    }
    break;


  case BCC_OK:
    if (byte == FLAG){
      *state = DONE;
    }
    else{
      *state = Start;
    }
    break;


  case DONE:
    break;
  }
}

struct termios oldtio,newtio;

// Opens a connection using the "port" parameters defined in struct linkLayer, returns "-1" on error and "1" on sucess
int llopen(linkLayer connectionParameters){


    //Connection stablishment

    /*
    Open serial port device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C.
    */

    fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY );
    if (fd < 0) {
        perror(connectionParameters.serialPort); 
        return(-1); 
    }

    signal(SIGALRM, atende);                     // Install the alarm routine

    if (tcgetattr(fd,&oldtio) == -1) { /* save current port settings */
        perror("tcgetattr");
        return(-1);
    }

    
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = connectionParameters.baudRate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME]    = 30;   /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 0;   /* blocking read until 5 chars received */

    /*
    VTIME e VMIN devem ser alterados de forma a proteger com um temporizador a
    leitura do(s) próximo(s) caracter(es)
    */


    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd,TCSANOW,&newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    //Connection established
    
    //llopen start
    char buf[255], buf1[1];
    unsigned char replyBuf[255], checkBuf[2];       // checkBuf terá valores de A e C para verificar BCC
    enum stateMachine state = Start;
    int res, STOP;

    switch(connectionParameters.role){
        case TRANSMITTER:            
            printf("TRANSMITTER HERE\n");
            buf[0]= FLAG; // F
            buf[1]= A_ER;                           // A
            buf[2]= C_SET;                          // 

            buf[3]= BCC(A_ER, C_SET);               // BCC
            buf[4]= FLAG;                           // F
            failed = TRUE;
            int STOP;
            res = write(fd,buf,SET_SIZE);
            alarm(3);
                   /* loop for input */
             while(STOP==FALSE){    
              if(failed == TRUE){
                res = write(fd,buf,SET_SIZE);
                  if(attempt>=connectionParameters.numTries){return-1;}
                    printf("%d bytes written\n", res);  
                    attempt++;
                    failed = FALSE;
                    state = Start;
                    STOP = FALSE;
                    printf("%#4.2x    and    %#4.2x    and    %#4.2x    and    %#4.2x    and    %#4.2x\n%#4.2x    and    %#4.2x\n", buf[0], buf[1], buf[2], buf[3], buf[4], A_ER, C_SET);
                    alarm(TIMEOUT_DEFAULT);
              }
                   /* loop for input */
                    res = read(fd,buf1,1);
                   if(res>0){
                      if(failed == TRUE) break;
                      printf("nº bytes lido: %d - ", res);
                      printf("content: %#4.2x\n", buf1[0]);

                    determineState(&state, checkBuf, buf1[0], buf, connectionParameters.role);
                    if (state == DONE) STOP=TRUE;
                   } 
              
             
            }
            
            alarm(0);
            printf("desliguei\n");
            break;
        
        case RECEIVER:
            printf("RECEIVER HERE\n");
            state = Start;
            STOP = FALSE;

            while (STOP==FALSE) {       /* loop for input */
              res = read(fd,buf,1);   /* returns after 1 char has been input */
              printf("nº bytes lido: %d - ", res);
              printf("content: %#4.2x\n", buf[0]);

              determineState(&state, checkBuf, buf[0], replyBuf, connectionParameters.role);

              if (state == DONE) STOP=TRUE;
            }

            replyBuf[0]= FLAG;                      // F
            replyBuf[1]= A_ER;                      // A
            replyBuf[2]= C_UA;                      // C
            replyBuf[3]= BCC(A_ER, C_UA);           // BCC
            replyBuf[4]= FLAG;                      // F
            
            res = write(fd,replyBuf,UA_SIZE);       //+1 para enviar o \0 
            printf("%d bytes written\n", res);      //res a contar com o \n e com o \0
            printf("%#4.2x    and    %#4.2x    and    %#4.2x    and    %#4.2x    and    %#4.2x\n%#4.2x    and    %#4.2x\n", replyBuf[0], replyBuf[1], replyBuf[2], replyBuf[3], replyBuf[4], checkBuf[0], checkBuf[1]);
            break;
    }
  return 1;
}




// Sends data in buf with size bufSize received from Application Layer (Transmitter) to the Link Layer and write it to the other ttySx
int llwrite(char* buf, int bufSize){ 
  newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
  newtio.c_cc[VMIN]     = 1;

  // printf("llwrite: INIT\n");
  if(bufSize < 0 || bufSize > MAX_PAYLOAD_SIZE){
    printf("Failure: bufSize are invalid\n");
    return -1;
  };
  // Information Frame has as header 4 bytes: F,A,C,BCC1; Data has bufSize bytes + 2 more bytes after the last data byte (BCC2 and F). In total it has 4+bufSize+2 bytes
  const int info_size = 2*bufSize+6;              // Worst case scenario, with all bytes stuffed
  unsigned char info[info_size];
  int res, stop=FALSE;
  enum stateMachine state = Start;
  char buf1[1];
  unsigned char checkBuf[2]; 
  
//while (!stop){


  info[0] = FLAG;
  info[1] = A_ER;                                 // A_RE is only used when Receiver sends DISC to Emitter
  info[2] = C_NS(s);
  info[3] = BCC(A_ER, C_NS(s));                   // BCC1
  unsigned char BCC2 = 0b00000000;

  

  int buf_read = 0;
  int info_read = 4;                              // To write all the data(inside buf) at the next info position
  int stuffing_count = 0;
 
  // Starting to read the actual data and doing the byte stuffing
  for(buf_read; buf_read < bufSize; buf_read++){
        // printf("buf[%d] = %#4.2x       BCC2 = %#4.2x\n", buf_read, buf[buf_read], BCC2);
       BCC2 = BCC(BCC2, buf[buf_read]);
      
      
       // BCC2 at the next position of the last data byte (Dn) read
    if (buf[buf_read] == FLAG){
      info[info_read] = ESC;
      info_read++;
      info[info_read] = FLAG ^ 0b00100000;
      info_read++;
      stuffing_count += 1;
    }
    else if (buf[buf_read] == ESC){
      info[info_read] = ESC;
      info_read++;
      info[info_read] = ESC ^ 0b00100000;
      info_read++;
      stuffing_count += 1;
    }
    else{
      info[info_read] = buf[buf_read];
      info_read++;
    }
  }
  info[info_read] = BCC2;
  printf("BCC2 is %#4.2x\n", BCC2);
  info_read += 1;                                    // Setting the correct number of info buffer as 4 + dataSize + Stuffing + 1(BCC) + 1(F)
  info[info_read] = FLAG;
 
  
  if(info_read > 2*bufSize+6){
    printf("Error: %d written characters, should be less then %d\n", info_read, 2*bufSize+6);
    return -1;
  }
  else if(info_read < bufSize){
    printf("Error: %d written characters, should be %d\n", info_read, bufSize-1);
    return -1;
  }
  else {
    // printf("llwrite: END\n");
    res = write(fd, info, info_read+1);               // Maybe it needs info_read+1
    if(res <= 0){
      perror("write: ");
    }
    // printf("res = %d\n", res);
    if(res > 0) return res;
    else {
      printf("llwrite has  invalid value\n");
      return -1;
    }
  }

}

// Receive data in packet from the other ttySx and send it back to Aplication Layer, which means sending the complete data to the receiver
int llread(char* packet){
//   newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
//   newtio.c_cc[VMIN]     = 1;
  
//   int res=0;
//   const int buf_size = (2*MAX_PAYLOAD_SIZE);
//   unsigned char aux_buf[1], buf_read[buf_size];
//   int buf_count = 0, read_count = 0, STOP = FALSE;
//   unsigned char BCC2check = 0b00000000;
 
//     while(1){
   
//       res = read(fd, aux_buf, 1);
//       buf_read[buf_count]=aux_buf[0];
     
//       if(buf_count > 1 && buf_read[buf_count] == FLAG) {printf("\nFlag\n"); buf_count += 1; break;}
//       buf_count += 1;
      
//     }
//     for(int i=4; i < buf_count-2; i++){
     
//       if(buf_read[i] == ESC && buf_read[i+1] == (FLAG ^ 0b00100000)){
//         // printf(">>>>>FLAG<<<<<\n");
//         packet[read_count] = FLAG;
//         i++; 
//         read_count += 1;
//       }
//       else if(buf_read[i] == ESC && buf_read[i+1] == (ESC ^ 0b00100000)){
//         // printf(">>>>>Esc<<<<<\n");
//         packet[read_count] = ESC;
//         i++;
//         read_count += 1;
//       }
//       else {
     
//       packet[read_count] = buf_read[i];
      
      

//         read_count += 1;
//       }
//       BCC2check = BCC(BCC2check, packet[read_count-1]); // read_count-1 because here we already add 1 to read_count
//     }

  
//     printf(">>>>>>>>>>>>>buf_count = %d<<<<<<<<<<<<<<<<<< \n", buf_count);
//     printf("BCC2check = %#4.2x       BCC2 = %#4.2x\n", BCC2check, buf_read[buf_count-2]);
//     if(BCC2check == buf_read[buf_count-2]) {
//       printf(" >>>>>>>>>>>>>>>>>>>    BCC2 OK\n");
//     }
//     else printf(" >>>>>>>>>>>>>>>>>>>    BCC2 NOT OKAY\n");

//   return read_count;



 
// }

// int llclose(linkLayer connectionParameters, int showStatistics){

//     char buf[255];
//     unsigned char DISC[6], UA[6];
//     DISC[0]=0X5C;
//     DISC[1]=0X01;
//     DISC[2]=0X0B;
//     DISC[3]=DISC[1] ^ DISC[2];
//     DISC[4]=0X5C;
//     DISC[5]=0;
//     char buf1[1];
//     unsigned char replyBuf[255], checkBuf[2];       // checkBuf terá valores de A e C para verificar BCC
//     enum stateMachine state = Start;
//     int res, STOP;

    

//     bzero(&newtio, sizeof(newtio));
//     newtio.c_cflag = connectionParameters.baudRate | CS8 | CLOCAL | CREAD;
//     newtio.c_iflag = IGNPAR;
//     newtio.c_oflag = 0;

//         /* set input mode (non-canonical, no echo,...) */
//     newtio.c_lflag = 0;

//     newtio.c_cc[VTIME]    = 1;   /* inter-character timer unused */
//     newtio.c_cc[VMIN]     = 0;   /* blocking read until 5 chars received */

//     tcflush(fd, TCIOFLUSH);

//     if (tcsetattr(fd,TCSANOW,&newtio) == -1) {
//         perror("tcsetattr");
//         exit(-1);
//     }

//     //printf("New termios structure set\n");

//     switch (connectionParameters.role)
//     {
//         case TRANSMITTER:
//             UA[0]=0X5C;
//             UA[1]=0X01;
//             UA[2]=0X07;
//             UA[3]=0X06;
//             UA[4]=0X5C;
//             UA[5]=0;

//             res = write(fd,DISC,sizeof(DISC));
//             //printf("%d bytes written\n", res);
//             alarm(TIMEOUT_DEFAULT);
//             attempt++;

//             //while para imprimir e esperar receber DISC
//             while(STOP==FALSE)
//             {
//                 if(failed)
//                 {
//                     if(attempt == (MAX_RETRANSMISSIONS_DEFAULT + 1))
//                     {
                             
                            
//                        sleep(1);
//                        if ( tcsetattr(fd,TCSANOW,&oldtio) == -1) {
//                        perror("tcsetattr");
//                        exit(-1);
//                         }
//                       close(fd);
//                        STOP = TRUE; 
//                       return -1;
                        
//                     }
//                     res = write(fd,DISC,6);
//                     printf("RETRANSMITE %d bytes written\n", res);
//                     attempt++;
//                     failed=FALSE;
//                     alarm(TIMEOUT_DEFAULT); 
//                 }
//                 res = read(fd,buf+state,1);
//                 if(res>0)
//                 {
//                     switch (state)
//                     {    
//                         case Start:
//                             if(buf[state] == 0x5C)
//                                 state=FLAG_RCV;
//                             break;
//                         case FLAG_RCV:
//                             if(buf[state] == 0x01)
//                                 state=A_RCV;
//                             else if (buf[state] != 0x5C)
//                                 state = Start;    
//                             break;
//                         case A_RCV:
//                             if(buf[state] == 0x0B)
//                                 state=C_RCV;
//                             else if (buf[state] == 0x5C)
//                                 state = FLAG_RCV;
//                             else
//                                 state=Start;
//                             break;
//                         case C_RCV:
//                             if(buf[state] == DISC[3])
//                                 state=BCC_OK;
//                             else if (buf[state] == 0x5C)
//                                 state = FLAG_RCV;
//                             else
//                                 state=Start;
//                             break;
//                         case BCC_OK:
//                             if(buf[state] == 0x5C)
//                             {
//                                 state=DONE;
//                                 STOP=TRUE;
//                             }
//                             else
//                                 state=Start;
//                             break;
//                     }
//                 }
//                 //printf("state %d, lido %d\n",state,res);
//             }
      
//             buf[5]=0;
//             printf("DISC\n");
//             //sleep(1);
//             res = write(fd,UA,sizeof(UA));
//             //printf("%d bytes written\n", res);
//             alarm(TIMEOUT_DEFAULT);
//             break;
        
//         case RECEIVER:

//             //while para imprimir e esperar receber DISC
//             while(STOP==FALSE)
//             {
//                 res = read(fd,buf+state,1);
//                 if(res>0)
//                 {
//                     switch (state)
//                     {
//                         case Start:
//                             if(buf[state] == 0x5C)
//                                 state=1;
//                             break;
//                         case 1:
//                             if(buf[state] == 0x01)
//                                 state=2;
//                             else if (buf[state] != 0x5C)
//                                 state = Start;    
//                             break;
//                         case 2:
//                             if(buf[state] == 0x0B)
//                                 state=3;
//                             else if (buf[state] == 0x5C)
//                                 state = 1;
//                             else
//                                 state=Start;
//                             break;
//                         case 3:
//                             if(buf[state] == DISC[3])
//                                 state=4;
//                             else if (buf[state] == 0x5C)
//                                 state = 1;
//                             else
//                                 state=Start;
//                             break;
//                         case 4:
//                             if(buf[state] == 0x5C)
//                             {
//                                 state=5;
//                                 STOP=TRUE;
//                             }
//                             else
//                                 state=Start;
//                             break;
//                     }
//                     //printf("state %d, lido %d\n",state,res);
//                 }
//             }
    
//             buf[5]=0;
//             printf("DISC\n");

//             res = write(fd,DISC,6);
//             if(res<0) 
//             {   
//               printf("entrei\n");
//                 sleep(1);
//                 if ( tcsetattr(fd,TCSANOW,&oldtio) == -1) {
//                     perror("tcsetattr");
//                     exit(-1);
//                 }
//                 close(fd);
//                 return -1;
//             }
//             //printf("%d bytes written\n", res);    

//             STOP=FALSE;
//             state=0;

//             //while para imprimir e esperar receber UA
//             while(STOP==FALSE)
//             {
//                 res = read(fd,buf+state,1);
//                 if(res>0)
//                 {
//                     switch (state)
//                     {    
//                         case 0:
//                             if(buf[state] == 0x5C)
//                                 state=1;
//                             break;
//                         case 1:
//                             if(buf[state] == 0x01)
//                                 state=2;
//                             else if (buf[state] != 0x5C)
//                                 state = 0;    
//                             break;
//                         case 2:
//                             if(buf[state] == 0x07)
//                                 state=3;
//                             else if (buf[state] == 0x5C)
//                                 state = 1;
//                             else
//                                 state=0;
//                             break;
//                         case 3:
//                             if(buf[state] == 0x06)
//                                 state=4;
//                             else if (buf[state] == 0x5C)
//                                 state = 1;
//                             else
//                                 state=0;
//                             break;
//                         case 4:
//                             if(buf[state] == 0x5C)
//                             {
//                                 state=5;
//                                 STOP=TRUE;
//                             }
//                             else
//                                 state=0;
//                             break;
//                     }
//                 }
//                 //printf("state %d, lido %d\n",state,res);
//                 buf[5]=0;
//             }
//             printf("UA\n");
//             break;

//     }
//     STOP=FALSE;
//     state=0;
//     failed= FALSE;
//     attempt = 0;
//     sleep(1);
//     if ( tcsetattr(fd,TCSANOW,&oldtio) == -1) {
//         perror("tcsetattr");
//         exit(-1);
//     }
//     close(fd);
//     printf("returned\n");
//     return 1;
}
