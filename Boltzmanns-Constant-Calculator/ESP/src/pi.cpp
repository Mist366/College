/*
pi.cpp

-Reads output of ADS8322 (adc) and writes it to a log file
-Called by master.py

Written and Designed by Michael Thompson
*/

#ifdef _WIN32
std::cout << "THIS IS NOT A RASPBERRY PI\n";
#endif

#include <pthread.h>
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <math.h>
#include <wiringPi.h>

using namespace std;

//functions
int detlimit(string);
void setPins();
void nsleep();
int sample();
int readDigital(string);

//file names
const char tmp_file_name[] = "go_between.tmp";
const char pinout_name[] = "pinout.v";
string out_file_name = "failed.log";

//file
ofstream log_file;

//thread start
bool go = true;
pthread_t thread;
static void* keyPressed(void*){
    while(go){
        if (cin.get() == ' ') {
            go = false;
        }
    }
    return 0;
}

void setupThread(){
    (void) pthread_create(&thread, 0, keyPressed, 0);
}

void endThread(){
    (void) pthread_join(thread, NULL);
}
//thread end

//variable
const unsigned int MAX_SIZE = 4294967295/65535;
unsigned int sample_size = MAX_SIZE;

//file set variables
//pins
int D0 = 0;
int D1 = 0;
int D2 = 0;
int D3 = 0;
int D4 = 0;
int D5 = 0;
int D6 = 0;
int D7 = 0;
int D8 = 0;
int D9 = 0;
int D10 = 0;
int D11 = 0;
int D12 = 0;
int D13 = 0;
int D14 = 0;
int D15 = 0;
int nRD = 0;
int BUSY = 0;
int nCONVST = 0;

int main(){
    //setup
    ifstream tmp_file;
    setupThread();
    wiringPiSetup();
    setPins();
    tmp_file.open(tmp_file_name);
    if (!tmp_file.is_open()) perror("Error tmp_file_name does not exist");
    getline(tmp_file, out_file_name);
    tmp_file.close();
    log_file.open(out_file_name);

    if( remove(tmp_file_name) != 0 ) {
        perror( "Error deleting file" );
    }
    
    //sampling
    int tmp = -1;
    int pointer = 0;
    unsigned int samples[sample_size]; //holds samples for averaging so that the size of the log does not get too large too fast
    unsigned int send_me = 0;
    while (go) {
        tmp = sample();
        if (tmp != -1){
            //holds samples
            samples[pointer++] = tmp;
            if (pointer >= sample_size) {
                pointer = 0;
                send_me = 0;
                for (int i = 0; i<sample_size; i++){
                    //compiles samples
                    send_me += samples[i];
                }
                //sends the average of the samples to the log
                //may have to square first
                log_file << (send_me/sample_size) << endl;
            }
        }
    }
    
    //exiting
    endThread();
    return 0;
}

int detlimit(string tmp){
    return stoi(tmp.substr(tmp.find("=")+1, tmp.find("\n")));
}

void setPins(){
    ifstream pinout;
    string tmp;
    int count = 0;
    pinout.open(pinout_name);

    //get pin numbers from pinout_name
    getline(pinout, tmp);
    D0 = detlimit(tmp);
    getline(pinout, tmp);
    D1 = detlimit(tmp);
    getline(pinout, tmp);
    D2 = detlimit(tmp);
    getline(pinout, tmp);
    D3 = detlimit(tmp);
    getline(pinout, tmp);
    D4 = detlimit(tmp);
    getline(pinout, tmp);
    D5 = detlimit(tmp);
    getline(pinout, tmp);
    D6 = detlimit(tmp);
    getline(pinout, tmp);
    D7 = detlimit(tmp);
    getline(pinout, tmp);
    D8 = detlimit(tmp);
    getline(pinout, tmp);
    D9 = detlimit(tmp);
    getline(pinout, tmp);
    D10 = detlimit(tmp);
    getline(pinout, tmp);
    D11 = detlimit(tmp);
    getline(pinout, tmp);
    D12 = detlimit(tmp);
    getline(pinout, tmp);
    D13 = detlimit(tmp);
    getline(pinout, tmp);
    D14 = detlimit(tmp);
    getline(pinout, tmp);
    D15 = detlimit(tmp);
    getline(pinout, tmp);
    nRD = detlimit(tmp);
    getline(pinout, tmp);
    BUSY = detlimit(tmp);
    getline(pinout, tmp);
    nCONVST = detlimit(tmp);

    //set sample size
    getline(pinout, tmp);
    if (detlimit(tmp) != -1) sample_size = detlimit(tmp);

    //inputs
    pinMode(D0, INPUT);
    pinMode(D1, INPUT);
    pinMode(D2, INPUT);
    pinMode(D3, INPUT);
    pinMode(D4, INPUT);
    pinMode(D5, INPUT);
    pinMode(D6, INPUT);
    pinMode(D7, INPUT);
    pinMode(D8, INPUT);
    pinMode(D9, INPUT);
    pinMode(D10, INPUT);
    pinMode(D11, INPUT);
    pinMode(D12, INPUT);
    pinMode(D13, INPUT);
    pinMode(D14, INPUT);
    pinMode(D15, INPUT);

    //outputs
    pinMode(BUSY, INPUT);
    pinMode(nRD,OUTPUT);
    pinMode(nCONVST,OUTPUT);
}

//does not do anything
void nsleep(int time){

}

//handles inturrupt
int sample(){
    bool recieve = false;
    if(digitalRead(BUSY) == 0){
        recieve = true;
        digitalWrite(nRD, 0);

        //delay 10 ns
        nsleep(10);

        //read sample
        int sample = readDigital("little");

        //start next conversion
        digitalWrite(nRD, 1);
        digitalWrite(nCONVST, 0);
        return sample;
    } else {
        //delay 20ns
        nsleep(20);
        digitalWrite(nCONVST, 1);
    }
    return -1;
}

int readDigital(string endian="little"){
    //Makes array size 16 filled with 0's
    int pins[16];

    //Reads inputs
    pins[0] = digitalRead(D0);
    pins[1] = digitalRead(D1);
    pins[2] = digitalRead(D2);
    pins[3] = digitalRead(D3);
    pins[4] = digitalRead(D4);
    pins[5] = digitalRead(D5);
    pins[6] = digitalRead(D6);
    pins[7] = digitalRead(D7);
    pins[8] = digitalRead(D8);
    pins[9] = digitalRead(D9);
    pins[10] = digitalRead(D10);
    pins[11] = digitalRead(D11);
    pins[12] = digitalRead(D12);
    pins[13] = digitalRead(D13);
    pins[14] = digitalRead(D14);
    pins[15] = digitalRead(D15);

    int total = 0;
    if (endian == "big"){ //D15 is least significant bit
            int y;
            for (int i = 0; i<16; i++) {
                y = 15-i;
                total += pins[i]*pow(2,y);
            }
    } else if (endian == "little"){ //D0 is least significant bit
            for (int i = 0; i<16; i++) {
                total += pins[i]*pow(2,i);
            }
    } else throw 20;
    return total;
    }