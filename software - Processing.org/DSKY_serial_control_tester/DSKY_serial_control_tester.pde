import processing.serial.*;


Integer NounVal = 0;
Integer VerbVal = 0;
Integer ProgVal = 0;
Integer TopVal = 0;
Integer MidVal = 0;
Integer BotVal = 0;
Integer SpecVal = 8;  //Special characters (Comp Acty, Verb, Noun, Prog, horiztonal lines)

int ProgTimer = 0;
int NounTimer = 0;
int VerbTimer = 0;
int angTimer = 0;

float tmpangle = 1.4;


Serial DSKYSerial;

void setup() {
  size(800, 800);  // create the window
  String[] list = Serial.list();
  delay(20);
  println("Serial Ports List:");
  println(list);
  
  DSKYSerial = new Serial(this, "COM13");
  
}


void draw() {
   
  
  background(255);
  fill(0);
  
  
  if (millis() - ProgTimer > 500)
    {
      ProgVal = (ProgVal > 99) ? 0 : ProgVal + 5;
      ProgTimer = millis();
    }
    
  if (millis() - NounTimer > 100)
    {
      NounVal = (NounVal > 99) ? 0 : NounVal + 2;
      NounTimer = millis();
    }
  
  if (millis() - VerbTimer > 30)
    {
      VerbVal = (VerbVal > 99) ? 0 : VerbVal + 1;
       VerbTimer = millis();
    }
  
  
  if( millis() - angTimer > 200)
    {
      tmpangle = (tmpangle > 6.283) ? 0 : tmpangle + 0.001;
      angTimer = millis();
    }
    
  
  TopVal = Math.round(sin(tmpangle) * 100000.0);
  MidVal = Math.round(cos(tmpangle) * 100000.0);
  textSize(26); 
  
  text("PROG",400,40);
  text("NOUN",400,140);
  text("VERB",100,140);
  text(DSKY_format_2dig(ProgVal),400,80);
  text(DSKY_format_2dig(NounVal),400,180);
  text(DSKY_format_2dig(VerbVal),100,180);
  text(DSKY_format_5dig(TopVal),200,270);
  text(DSKY_format_5dig(MidVal),200,350);
  text(DSKY_format_5dig(BotVal),200,430);
  
  DSKYSerial.write("[" + DSKY_format_2dig(ProgVal) + DSKY_format_2dig(VerbVal) +
                          DSKY_format_2dig(NounVal) + DSKY_format_5dig(TopVal) +
                          DSKY_format_5dig(MidVal)  + DSKY_format_5dig(BotVal) +
                          "8");
  
  text("[" + DSKY_format_2dig(ProgVal) + DSKY_format_2dig(VerbVal) +
                          DSKY_format_2dig(NounVal) + DSKY_format_5dig(TopVal) +
                          DSKY_format_5dig(MidVal)  + DSKY_format_5dig(BotVal) +
                          "8",20,600);
  
}


String DSKY_format_5dig(Integer intval)
  {
    if( intval > 99999)
      {
        intval = 99999;
      }
    else if( intval < -99999)
      {
        intval = -99999;
      }
      
      
    return ((intval < 0) ? "-" : "+") + String.format("%05d", Math.abs(intval)); 
  }
  
 String DSKY_format_2dig(Integer intval)
  {
    if (intval < 0)
      {
        return "00";
      }
    else if ( intval > 99)
      {
        return "99";
      }
      
    else 
      {
        return String.format("%02d", intval);
      }
  }
