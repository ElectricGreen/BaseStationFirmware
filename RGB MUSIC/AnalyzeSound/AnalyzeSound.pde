import g4p_controls.*;

/**
  * This sketch demonstrates how to use an FFT to analyze
  * the audio being generated by an AudioPlayer.
  * <p>
  * FFT stands for Fast Fourier Transform, which is a 
  * method of analyzing audio that allows you to visualize 
  * the frequency content of a signal. You've seen 
  * visualizations like this before in music players 
  * and car stereos.
  * <p>
  * For more information about Minim and additional features, 
  * visit http://code.compartmental.net/minim/
  */
import processing.serial.*;
import ddf.minim.analysis.*;
import ddf.minim.*;
import ddf.minim.effects.*;

// The serial port:
Serial myPort;       
  
Minim       minim;
AudioPlayer jingle;
FFT         fft;
AudioInput input;



LowPassSP lowpass;
HighPassSP highpass;

public byte mode = 0;
int rMax = 252;
int gMax = 252;
int bMax = 252;

long lastTime = 0;
float total = 0;
long samples = 0;
boolean init = false;
int filtro;

void setup()
{
  size(1024, 750, P3D);
  
  createGUI();
  
  minim = new Minim(this);
  
  // loop the file indefinitely
  //jingle.loop();
  
  // create an FFT object that has a time-domain buffer 
  // the same size as jingle's sample buffer
  // note that this needs to be a power of two 
  // and that it means the size of the spectrum will be half as large.
  //fft = new FFT( jingle.bufferSize(), jingle.sampleRate() );
  input = minim.getLineIn(Minim.STEREO, 2048); 

  
  // hipass = new HighPassSP(440,song.sampleRate());
 // lowpass = new LowPassSP(50,input.sampleRate());
 // bandpass = new BandPass(440,20,input.sampleRate());
   
  //input.addEffect(lowpass);
  
   //fft = new FFT(input.bufferSize(), input.sampleRate()); 
  //fft.window(FFT.HAMMING);
// Open the port you are using at the rate you want:
myPort = new Serial(this, Serial.list()[0], 9600);

//input = minim.getLineIn(Minim.STEREO, 1024); 

  lowpass = new LowPassSP(20000, 44100);     //DICHIARAZIONE FILTRO PASSA BASSO
  highpass = new HighPassSP(200, 44100);   //DICHIARAZIONE FILTRO PASSA ALTO
  input.addEffect(lowpass);
 input.addEffect(highpass);
  
  fft = new FFT(input.bufferSize(), input.sampleRate());
 // fft.linAverages(30);
  fft.logAverages(21, 12);

 
  filtro=0; //0=none, 1=bass, 2=high

    init = true;
}

void keyPressed() {
  if(key=='l' || key=='L'){ //introduco filtro passa basso
    if(filtro==0 || filtro==2){
        highpass.setFreq(200); //tolgo filtro passa alto
        lowpass.setFreq(200);  //introduco filtro passa basso
        filtro=1;
    }
  }
  else if(key=='h' || key=='H'){ //introduco filtro passa alto
    if(filtro==0 || filtro==1){
        highpass.setFreq(10000); //introduco filtro passa alto
        lowpass.setFreq(20000);  //tolgo filtro passa basso
        filtro=2;
    }
  }
  else if(key=='n' || key=='N'){ //tolgo ogni filtro (canzone normale)
    if(filtro==1 || filtro==2){
        highpass.setFreq(200);   //tolgo filtro passa alto
        lowpass.setFreq(20000);  //tolgo filtro passa basso
        filtro=0;
    }
  }
}

void draw()
{
  if (init){
        delay(100);
        
    myPort.write(0xFF);
    delay(100);
    myPort.write(0);
        delay(100);
        
    myPort.write(0xFE);
        delay(100);
    myPort.write(0);
        delay(100);

    myPort.write(0xFD);
        delay(100);
    myPort.write(0);
        delay(100);
    myPort.write(0xFF);
            delay(100);
    myPort.write(0);
        delay(5000);
            myPort.write(0xFD);
                delay(100);

    init = false;
  }
  
  switch(mode){
    case 0:
    //off
    break;
    
    case 1: //Music
  background(0);
  
  // perform a forward FFT on the samples in jingle's mix buffer,
  // which contains the mix of both the left and right channels of the file
  fft.forward( input.mix );
  int r = 0;
  int g = 0;
  int b = 0;
int size = (fft.avgSize());
//(6/16) before no avg
  for(int i = 0; i < size; i++)
  {
    if(i<(size/18)*7){
      r+=fft.getAvg(i);
      stroke(#FF0000);
    }else if(i<((size/18)*12)){
      stroke(#00FF00);
      g+=fft.getAvg(i);
    }else {
      stroke(#0000FF);
      b+=fft.getAvg(i);
    }
    // draw the line for frequency band i, scaling it up a bit so we can see it
    line( i*4, height,  i*4, height - fft.getAvg(i)*8 );
    line( (i*4)+1, height, (i*4)+1, height - fft.getAvg(i)*8 );
    line( (i*4)+2, height, (i*4)+2, height - fft.getAvg(i)*8 );
    line( (i*4)+3, height, (i*4)+3, height - fft.getAvg(i)*8 );

  }
  if (millis()>lastTime+60){
    lastTime = millis();
    //println("Value R:"+r+"/n");
  //println("Value G:"+g+"/n");
  //println("Value B:"+b+"/n");
  
 int sendR = (int)map(r, 0, 2500, 0 ,rMax);
    int sendG = (int)map(g, 0, 2500, 0 ,gMax);
  int sendB = (int)map(b, 0, 2500, 0 ,bMax);

 // println("send R:"+sendR+"/n");
 // println("send G:"+sendG+"/n");
  //println("send B:"+sendB+"/n");
  //#define RGB_RED    3
//#define RGB_GREEN  5
//#define RGB_BLUE   6
  if (sendR>0xFC){
    sendR = 0xFC;
    println("over");
  }
   if (sendG>0xFC){
    sendG = 0xFC;
    println("over");
  }
   if (sendB>0xFC){
    sendB = 0xFC;
    println("over");
  }
  
   myPort.write(0xFF);
   myPort.write((byte)sendR);
   
   myPort.write(0xFE);
   myPort.write((byte)sendG);
   
   myPort.write(0xFD);
   myPort.write((byte)sendB);
  }
  break;
  
  case 2: //manual
  break;
  }
}
