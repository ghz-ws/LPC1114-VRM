#include "mbed.h"

I2C i2c(P0_5,P0_4);     // sda, scl
BufferedSerial uart0(P1_7, P1_6,9600);  //TX, RX, 115200
SPI spi(P0_9, P0_8, P0_6);    //mosi, miso, sclk
DigitalOut cs1(P0_7);   //adc
DigitalIn drdy1(P0_11);
DigitalIn sw(P1_5);
DigitalOut shunt1(P0_2);
DigitalOut shunt2(P0_3);

//lcd
const uint8_t contrast=40;  //lcd contrast
const uint8_t lcd_addr=0x7C;   //lcd i2c addr 0x7C
void lcd_init(uint8_t addr, uint8_t contrast);     //lcd init func
void char_disp(uint8_t addr, uint8_t position, char data);
void val_disp(uint8_t addr, uint8_t position, uint8_t digit,uint16_t val);

//adc control
const uint8_t rst=0b0110;
const uint8_t wreg=0b0100;
const uint8_t start=0b1000;

uint8_t buf[2];     //spi receive buf
int16_t raw_val;
float vm_f;
int16_t vm;    //mV unit

//oR-meter
void r_set(uint8_t range);
uint8_t range;
float dut_f;    //ohm unit
uint32_t dut_r; //ohm unit

//constants
#define adc_res 32768 //2^15 adc resolution. unipolar
#define adc_ref 2048   //2.048V adc reference
#define att 0.183   //vm att. 33/180
#define sens_R 1   //sens R

//calib
#define vm_gain 1.0022467    //vm1 gain calib
#define vm_ofs 7    //vm1 ofs calib
#define gain_100 0.9933    //100ohm gain calib
#define gain_1k 0.984203    //1kohm gain calib
#define gain_10k 1.0001011    //10kohm gain calib
#define gain_100k 1.015477    //100kohm gain calib

int main(){
    spi.format(8,1);
    spi.frequency(1000000); //SPI clk 1MHz
    i2c.frequency(400000);  //I2C clk 400kHz
    cs1=1;
    sw.mode(PullUp);
    shunt1=0;
    shunt2=0;

    //init LCD
    thread_sleep_for(100);  //wait for LCD power on
    lcd_init(lcd_addr, contrast);
    thread_sleep_for(100);  //wait for LCD power on
    
    printf("start\n\r");
    //adc init
    cs1=0;
    spi.write(rst);
    cs1=1;
    thread_sleep_for(10);   //NEED!!
    cs1=0;
    spi.write((wreg<<4)+(0x01<<2)+0);    //write addr 0x01, 1byte
    spi.write((0b000<<5)+(0b10<<3)+(0b1<<2));       //40sps, turbo mode, cc mode
    cs1=1;
    
    while (true){
        if(sw==0){  //V-meter
            char_disp(lcd_addr,3,'.');
            char_disp(lcd_addr,7,' ');
            char_disp(lcd_addr,8,'V');
            char_disp(lcd_addr,9,' ');
            char_disp(lcd_addr,10,' ');
            char_disp(lcd_addr,11,' ');
            char_disp(lcd_addr,12,' ');
            char_disp(lcd_addr,0x40+4,' ');
            char_disp(lcd_addr,0x40+5,' ');
            char_disp(lcd_addr,0x40+6,' ');
            char_disp(lcd_addr,0x40+7,' ');
            char_disp(lcd_addr,0x40+8,' ');
            char_disp(lcd_addr,0x40+9,' ');
            char_disp(lcd_addr,0x40+10,' ');
            cs1=0;
            spi.write((wreg<<4));       //write addr 0x00, 1byte
            spi.write((0b0000<<4)+1);   //ch0-1 mux, pga disable. vm
            cs1=1;
            while(true){
                while(true) if(drdy1==0) break;
                cs1=0;
                buf[1]=spi.write(0x00);
                buf[0]=spi.write(0x00);
                cs1=1;
                raw_val=(buf[1]<<8)+buf[0];
                vm_f=(float)raw_val*adc_ref/adc_res/att*vm_gain+vm_ofs;
                vm=(int16_t)vm_f;
                printf("Voltage: %d mV\n\r",vm);
                if(vm>=0) char_disp(lcd_addr,0,'+');
                else{
                    char_disp(lcd_addr,0,'-');
                    vm=abs(vm);
                }
                val_disp(lcd_addr,1,2,vm/1000);
                val_disp(lcd_addr,4,3,vm-(vm/1000)*1000);
                if(vm>=11100){
                    char_disp(lcd_addr,0x40+0,'O');
                    char_disp(lcd_addr,0x40+1,'V');
                    char_disp(lcd_addr,0x40+2,'L');
                    char_disp(lcd_addr,0x40+3,'D');
                }else{
                    char_disp(lcd_addr,0x40+0,' ');
                    char_disp(lcd_addr,0x40+1,' ');
                    char_disp(lcd_addr,0x40+2,' ');
                    char_disp(lcd_addr,0x40+3,' ');
                }
                if(sw==1)break;
            }
        }else{  //R-meter
            char_disp(lcd_addr,2,' ');
            char_disp(lcd_addr,6,'.');
            char_disp(lcd_addr,10,' ');
            char_disp(lcd_addr,11,'k');
            char_disp(lcd_addr,12,0x1e);
            cs1=0;
            spi.write((wreg<<4));       //write addr 0x00, 1byte
            spi.write((0b1011<<4)+0);   //AIN3-GND mux, pga enable
            cs1=1;
            thread_sleep_for(10);   //NEED!!
            cs1=0;
            spi.write((wreg<<4)+(0x03<<2)+0);    //write addr 0x03, 1byte
            spi.write((0b011<<5));       //IDAC1 connect to AIN2
            cs1=1;
            while(true){
                while(true){    //range search
                    r_set(range);
                    while(true) if(drdy1==0) break;
                    while(true) if(drdy1==0) break;
                    while(true) if(drdy1==0) break;
                    cs1=0;
                    buf[1]=spi.write(0x00);
                    buf[0]=spi.write(0x00);
                    cs1=1;
                    raw_val=(buf[1]<<8)+buf[0];
                    vm_f=(float)raw_val*adc_ref/adc_res;
                    vm=(int16_t)vm_f;

                    if(range==0&&vm>660)range=1;
                    else if(range==1&&vm<165)range=0;
                    else if(range==1&&vm>1650)range=2;
                    else if(range==2&&vm<110)range=1;
                    else if(range==2&&vm>1100)range=3;
                    else if(range==3&&vm<740)range=2;
                    else break;
                }

                if(range==0)dut_f=(vm_f/4)/1.5*gain_100; //ohm unit. 100ohm range
                else if(range==1)dut_f=(vm_f/1)/1.5*gain_1k; //ohm unit. 1kohm range
                else if(range==2)dut_f=(vm_f/1)/0.1*gain_10k; //ohm unit. 10kohm range
                else if(range==3)dut_f=(vm_f/2)/0.05; //ohm unit. 100kohm range
                if(range==3)dut_f=(dut_f*20000)/(20000-dut_f)*gain_100k;    //20kohm shunt. calc
                dut_r=(uint32_t)dut_f;

                val_disp(lcd_addr,0,2,dut_r/1000000);
                val_disp(lcd_addr,3,3,(dut_r-(dut_r/1000000)*1000000)/1000);
                val_disp(lcd_addr,7,3,dut_r%1000);

                if(range==0){
                    char_disp(lcd_addr,0x40+0,'1');
                    char_disp(lcd_addr,0x40+1,'0');
                    char_disp(lcd_addr,0x40+2,'0');
                    char_disp(lcd_addr,0x40+3,0x1e);
                    char_disp(lcd_addr,0x40+4,'R');
                    char_disp(lcd_addr,0x40+5,' ');
                    printf("100ohm Range, Voltage: %d mV, Current: 1.5mA, Resistance: %d ohm\n\r",vm/4,dut_r);
                }else if(range==1){
                    char_disp(lcd_addr,0x40+0,'1');
                    char_disp(lcd_addr,0x40+1,'k');
                    char_disp(lcd_addr,0x40+2,0x1e);
                    char_disp(lcd_addr,0x40+3,'R');
                    char_disp(lcd_addr,0x40+4,' ');
                    char_disp(lcd_addr,0x40+5,' ');
                    printf("1kohm Range, Voltage: %d mV, Current: 1.5mA, Resistance: %d ohm\n\r",vm,dut_r);
                }else if(range==2){
                    char_disp(lcd_addr,0x40+0,'1');
                    char_disp(lcd_addr,0x40+1,'0');
                    char_disp(lcd_addr,0x40+2,'k');
                    char_disp(lcd_addr,0x40+3,0x1e);
                    char_disp(lcd_addr,0x40+4,'R');
                    char_disp(lcd_addr,0x40+5,' ');
                    printf("10kohm Range, Voltage: %d mV, Current: 100uA, Resistance: %d ohm\n\r",vm,dut_r);
                }else if(range==3){
                    char_disp(lcd_addr,0x40+0,'1');
                    char_disp(lcd_addr,0x40+1,'0');
                    char_disp(lcd_addr,0x40+2,'0');
                    char_disp(lcd_addr,0x40+3,'k');
                    char_disp(lcd_addr,0x40+4,0x1e);
                    char_disp(lcd_addr,0x40+5,'R');
                    printf("100kohm Range, Voltage: %d mV, Current: 50uA(20kohm shunt), Resistance: %d ohm\n\r",vm/2,dut_r);
                }
                if(dut_r>=10000000){
                    char_disp(lcd_addr,0x40+7,'O');
                    char_disp(lcd_addr,0x40+8,'V');
                    char_disp(lcd_addr,0x40+9,'L');
                    char_disp(lcd_addr,0x40+10,'D');
                }else{
                    char_disp(lcd_addr,0x40+7,' ');
                    char_disp(lcd_addr,0x40+8,' ');
                    char_disp(lcd_addr,0x40+9,' ');
                    char_disp(lcd_addr,0x40+10,' ');
                }
                if(sw==0)break;
            }
        }
    }
}

void r_set(uint8_t range){
    if(range==0){   //100ohm
        cs1=0;
        spi.write((wreg<<4));       //write addr 0x00, 1byte
        spi.write((0b1011<<4)+(0b010<<1)+0);   //AIN3-GND mux, pga x4(0b010)
        cs1=1;
        thread_sleep_for(10);   //NEED!!
        cs1=0;
        spi.write((wreg<<4)+(0x02<<2)+0);    //write addr 0x02, 1byte
        spi.write(0b111);       //IDAC1 1500uA(0b111)
        cs1=1;
        shunt1=0;
    }else if(range==1){ //1kohm
        cs1=0;
        spi.write((wreg<<4));       //write addr 0x00, 1byte
        spi.write((0b1011<<4)+(0b000<<1)+0);   //AIN3-GND mux, pga x1(0b000)
        cs1=1;
        thread_sleep_for(10);   //NEED!!
        cs1=0;
        spi.write((wreg<<4)+(0x02<<2)+0);    //write addr 0x02, 1byte
        spi.write(0b111);       //IDAC1 1500uA(0b111)
        cs1=1;
        shunt1=0;
    }else if(range==2){ //10kohm
        cs1=0;
        spi.write((wreg<<4));       //write addr 0x00, 1byte
        spi.write((0b1011<<4)+(0b000<<1)+0);   //AIN3-GND mux, pga x1(0b000)
        cs1=1;
        thread_sleep_for(10);   //NEED!!
        cs1=0;
        spi.write((wreg<<4)+(0x02<<2)+0);    //write addr 0x02, 1byte
        spi.write(0b011);       //IDAC1 100uA(0b011)
        cs1=1;
        shunt1=0;
    }else{ //100kohm over
        cs1=0;
        spi.write((wreg<<4));       //write addr 0x00, 1byte
        spi.write((0b1011<<4)+(0b001<<1)+0);   //AIN3-GND mux, pga x2(0b001)
        cs1=1;
        thread_sleep_for(10);   //NEED!!
        cs1=0;
        spi.write((wreg<<4)+(0x02<<2)+0);    //write addr 0x02, 1byte
        spi.write(0b010);       //IDAC1 50uA(0b010)
        cs1=1;
        shunt1=1;
    }
}

//OLED init func
void oled_init(uint8_t addr, uint8_t contrast){
    char buf[2];
    buf[0] = 0x0;
    buf[1]=0x01;           //0x01 clear disp
    i2c.write(addr,buf,2);
    thread_sleep_for(20);
    buf[1]=0x02;           //0x02 return home
    i2c.write(addr,buf,2);
    thread_sleep_for(20);
    buf[1]=0x0C;           //0x0c disp on
    i2c.write(addr,buf,2);
    thread_sleep_for(20);
    buf[1]=0x01;           //0x01 clear disp
    i2c.write(addr,buf,2);
    thread_sleep_for(20);

    //set contrsat
    buf[1]=0x2a;
    i2c.write(addr,buf,2);
    buf[1]=0x79;    //SD=1
    i2c.write(addr,buf,2);
    buf[1]=0x81;    //contrast set
    i2c.write(addr,buf,2);
    buf[1]=contrast;    //contrast value
    i2c.write(addr,buf,2);
    buf[1]=0x78;    //SD=0
    i2c.write(addr,buf,2);
    buf[1]=0x28;    //0x2C, 0x28
    i2c.write(addr,buf,2);
}

//disp char func
void char_disp(uint8_t addr, uint8_t position, char data){
    char buf[2];
    buf[0]=0x0;
    buf[1]=0x80+position;   //set cusor position (0x80 means cursor set cmd)
    i2c.write(addr,buf,2);
    buf[0]=0x40;            //write command
    buf[1]=data;
    i2c.write(addr,buf,2);
}

//disp val func
void val_disp(uint8_t addr, uint8_t position, uint8_t digit, uint16_t val){
    char buf[2],data[4];
    uint8_t i;
    buf[0]=0x0;
    buf[1]=0x80+position;       //set cusor position (0x80 means cursor set cmd)
    i2c.write(addr,buf,2);
    data[3]=0x30+val%10;        //1
    data[2]=0x30+(val/10)%10;   //10
    data[1]=0x30+(val/100)%10;  //100
    data[0]=0x30+(val/1000)%10; //1000
    buf[0]=0x40;                //write command
    for(i=0;i<digit;++i){
        if(i==0&&data[0]==0x30&&digit==4) buf[1]=0x20;
        else buf[1]=data[i+4-digit];
        i2c.write(addr,buf,2);
    }
}

//LCD init func
void lcd_init(uint8_t addr, uint8_t contrast){
    char lcd_data[2];
    lcd_data[0]=0x0;
    lcd_data[1]=0x38;
    i2c.write(addr,lcd_data,2);
    lcd_data[1]=0x39;
    i2c.write(addr,lcd_data,2);
    lcd_data[1]=0x14;
    i2c.write(addr,lcd_data,2);
    lcd_data[1]=0x70|(contrast&0b1111);
    i2c.write(addr,lcd_data,2);
    lcd_data[1]=0x56|((contrast&0b00110000)>>4);
    i2c.write(addr,lcd_data,2);
    lcd_data[1]=0x6C;
    i2c.write(addr,lcd_data,2);
    thread_sleep_for(200);
    lcd_data[1]=0x38;
    i2c.write(addr,lcd_data,2);
    lcd_data[1]=0x0C;
    i2c.write(addr,lcd_data,2);
    lcd_data[1]=0x01;
    i2c.write(addr,lcd_data,2);
}
