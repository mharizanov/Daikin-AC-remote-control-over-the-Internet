#include <EtherCard.h>
#include <avr/eeprom.h>
#include <Time.h>  
uint32_t lastUpdate = 0;
uint32_t time =0L;

// Default NTP client
int ntpclientportL = 0;

#define FW_VER 1
#define FW_SUBVER 0

#define TIME_REDIRECT_DELAY 2

#define DEBUG   1   // set to 1 to display free RAM on web page
#define SERIAL  1   // set to 1 to show incoming requests on serial port

#include <IRremote.h>

// # of bytes per command
const int COMMAND_LENGTH = 27;    

unsigned char daikin[COMMAND_LENGTH]     = { 
0x11,0xDA,0x27,0xF0,0x00,0x00,0x00,0x20,
//0    1    2   3    4    5     6   7
0x11,0xDA,0x27,0x00,0x00,0x41,0x1E,0x00,
//8    9   10   11   12    13   14   15
0xB0,0x00,0x00,0x00,0x00,0x00,0x00,0xC0,0x00,0x00,0xE3 };
//16  17    18  19   20    21   22  23   24   25   26
/*


byte 13=mode
b7 = 0
b6+b5+b4 = Mode
b3 = 0
b2 = OFF timer set
b1 = ON timer set
b0 = Air Conditioner ON

Modes: b6+b5+b4
011 = Cool
100 = Heat (temp 23)
110 = FAN (temp not shown, but 25)
000 = Fully Automatic (temp 25)
010 = DRY (temp 0xc0 = 96 degrees c)

byte 14=temp*2

byte 16=Fan
FAN control
b7+b6+b5+b4 = Fan speed
b3+b2+b1+b0 = Swing control up/down

Fan: b7+b6+b5+b4
0×30 = 1 bar
0×40 = 2 bar
0×50 = 3 bar
0×60 = 4 bar
0×70 = 5 bar
0xa0 = Auto
0xb0 = Not auto, moon + tree
Swing control up/down:
0000 = Swing up/down off
1111 = Swing up/down on
Swing control left/right:
0000 = Swing left/right off
1111 = Swing left/right on
*/

IRsend irsend;




#define CONFIG_EEPROM_ADDR ((byte*) 0x10)
#define CONFIG_USEDHCP 0
#define CONFIG_IP 1
#define CONFIG_GW 5
#define CONFIG_DNS 9
#define CONFIG_NTP 13  //32 bytes
#define CONFIG_TIMEZONEOFFSET 45
#define CONFIG_HTTPPORT 46
#define CONFIG_HTTPAUTH 48
#define CONFIG_HTTPJQENABLE 49
#define CONFIG_DDNSENABLE 50
#define CONFIG_DDNSUSER 51  //15 char
#define CONFIG_DDNSPASS 66  //15 char
#define CONFIG_HTTPUSER 81 //10 char
#define CONFIG_HTTPPASS 91 //10 char
#define CONFIG_DDNSHOST 101  //32 char
#define CONFIG_WANIP 133
#define CONFIG_VALID 137


const char map64[] PROGMEM =
{
    0x3e, 0xff, 0xff, 0xff, 0x3f, 0x34, 0x35, 0x36,
    0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x01,
    0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
    0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11,
    0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1a, 0x1b,
    0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b,
    0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33
};

const char b64[] PROGMEM = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/" ;

byte ntp[4]; //= { 204,9,54,119}; //={'n','t','p','.','y','o','u','r','.','o','r','g',0x00};


#define IRSTATE_EEPROM_ADDR ((byte*) 0x100)

struct IRState {
byte mode;
byte temp;
byte fan;
byte aux;
byte state;
byte enabled;
byte sched;
byte hour;
byte minutes;
long lastused;
} irstate;


#define ledPin 6      // the number of the LED pin
#define switchPin 5      // the number of the button pin

// Variables will change:
byte ledState = LOW;             // ledState used to set the LED
long previousMillis = 0;        // will store last time LED was updated
// the follow variables is a long because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
#define interval 500           // interval at which to blink (milliseconds)

//timer vars
#define numtimers 10
long previousMillisTimer = 0;        
long previousMillisExtIP = 0;        
#define intervalTimer 1000 * 30           //30 sec
const long intervalextip = 5*60*1000;          //5 min
const long authorizationInterval = 60*60*1000;   // time to keep the user authorized from that IP
long authorizedT = 0;

byte authorizedIP[] = { 0,0,0,0 };
byte extip[] = { 0,0,0,0 };

char DDNSchecksite[] PROGMEM = "checkip.dyndns.org";
char DDNSupdatesite[] PROGMEM = "members.dyndns.org";
char DDNSupdatesite2[] PROGMEM = "members.dyndns.org\r\nAuthorization: Basic ";

static BufferFiller bfill;  // used as cursor while filling the buffer


byte Ethernet::buffer[1100];   // tcp/ip send and receive buffer

byte loadConfig(int configopt) {
    return eeprom_read_byte(CONFIG_EEPROM_ADDR + configopt);
}

static void loadConfig2(byte *var, int configopt,int len) {
  for (byte i = 0; i < len; i++)
        var[i] = eeprom_read_byte(CONFIG_EEPROM_ADDR + configopt +i);
}


static void saveConfig(int configopt,byte configval) {
        eeprom_write_byte(CONFIG_EEPROM_ADDR + configopt, configval);
}

static void saveConfig2(byte *var,int configopt,int len) {
      for (byte i = 0; i < len; i++)
        eeprom_write_byte(CONFIG_EEPROM_ADDR + configopt +i, var[i]);   //((byte*) &var)[i]
}

static void loadIRstate(int offset) {
      for (byte i = 0; i < sizeof irstate; i++)
        ((byte*) &irstate)[i] = eeprom_read_byte(IRSTATE_EEPROM_ADDR + (offset*sizeof irstate) + i);
  
}

static void saveIRstate(int offset) {
    for (byte i = 0; i < sizeof irstate; i++)
        eeprom_write_byte(IRSTATE_EEPROM_ADDR + (offset*sizeof irstate) + i, ((byte*) &irstate)[i]);
}



#if DEBUG
static int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
#endif

static void gotPinged(byte* ptr) {
//ether.printIp(">>> ping from: ",ptr);
}

char okHeader[] PROGMEM = 
    "HTTP/1.0 200 OK\r\n"
    "Content-Type: text/html\r\n"
    "Pragma: no-cache\r\n"
;

char okHeaderjs[] PROGMEM = 
    "HTTP/1.0 200 OK\r\n"
    "Content-Type: application/x-javascript\r\n"
    "Pragma: no-cache\r\n"
    "\r\n"
;

char rdHeader[] PROGMEM = 
     "HTTP/1.0 302 found\r\n"
     "Location: /\r\n"
     "\r\n"
;                


char rdHTML[] PROGMEM = "\r\n<meta http-equiv=\"refresh\" content=\"$D; url=/$S$S\">Saved.. returning in $D seconds...";

char jqHTML[] PROGMEM =
"<!DOCTYPE html>"
"<html>"
	"<head>"
	"<title>Remote Control</title>"
	"<meta name='viewport' content='width=device-width, initial-scale=1'>"
	"<link rel='stylesheet' href='http://code.jquery.com/mobile/1.0/jquery.mobile-1.0.min.css' />"
	"<script src='http://code.jquery.com/jquery-1.6.4.min.js'></script>"
	"<script src='http://code.jquery.com/mobile/1.0/jquery.mobile-1.0.min.js'></script>"
        "</head>"
"<body>"
;

char njqHTML[] PROGMEM =
"<!DOCTYPE html>"
"<html>"
	"<head>"
	"<title>Remote Control</title>"
	"<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "</head>"
"<body>"
;


// Javascript for printing values in the homepage
prog_uchar js0[] PROGMEM =
    "function w(s){document.writeln(s)}\n"
    "w(\"Mode:<select data-native-menu='false' id=d name=d><option value='4'>Heat</option><option value='3'>Cool</option><option value='0'>Auto</option><option value='2'>Dry</option></select><br>Temp:<select name=t id=t>\")\n"
    "for(i=14;i<32;i++){w(\"<option value='\"+i+\"'>\"+i+\"</option>\");}\n"
    "w(\"</select><br>\")\n"
    "w(\"Fan:<select data-native-menu='false' id=f name=f><option value='176'>Night</option><option value='160'>Auto</option><option value='48'>*</option><option value='80'>***</option><option value='112'>*****</option></select><br>\")\n"
    "w(\"Aux:<select data-native-menu='false' name=a id=a><option value='0'>Normal</option><option value='1'>Powerful</option><option value='16'>Silent</option></select><br>\")\n"
    "w(\"State:<select name=s id=s data-role='slider' ><option value='1'>On</option><option value='0'>Off</option></select><br>\");\n"
;

prog_uchar js1[] PROGMEM =
        "function w(s){document.writeln(s)}\n"
       	"w(\"<button type='button' onClick=\\\"parent.location='/'\\\">Back</button>\");\n"     
	"w(\"<button type='submit'>Set</button>\");";


prog_uchar js2[] PROGMEM =
    "function w(s){document.writeln(s)}\n"
    "w(\"Go to timer # <br>\");"
    "for(i=0;i<10;i++){w(\"<a href='s\"+i+\"' data-ajax='false'>\"+i+\"</a>&nbsp;&nbsp;\");}"
    "w(\"<br>Enabled?<select id=e name=e data-role='slider'><option value='0'>No</option><option value='1'>Yes</option></select>\")\n"
    "w(\"<br>Day:<select data-native-menu='false' id=c name=c><option value='127'>MTWTFSS</option><option value='31'>MTWTF</option><option value='96'>SS</option><option value='1'>Mon</option><option value='2'>Tue</option><option value='4'>Wed</option><option value='8'>Thu</option><option value='16'>Fri</option><option value='32'>Sat</option><option value='64'>Sun</option></select>\")\n"
    "w(\"<br>Hour:<select id=h name=h>\")\n"
    "for(i=0;i<24;i++){w(\"<option value='\"+i+\"'>\"+i+\"</option>\");}"
    "w(\"</select><br>Min:<select data-native-menu='false' id=m name=m><option value='0'>00</option><option value='15'>15</option><option value='30'>30</option><option value='45'>45</option></select></fieldset><br>\")\n"
;


prog_uchar js3[] PROGMEM =
"function sv(e,v) {\n"
"document.getElementById(e).value=v;}\n"
"function svi(v1,v2,v3,v4,v5) {\n"
"sv('d',v1);"
"sv('t',v2);"
"sv('f',v3);"
"sv('a',v4);"              
"sv('s',v5);"  
"}\n"
"function svt(v1,v2,v3,v4) {\n"
"sv('e',v1);"
"sv('c',v2);"
"sv('h',v3);"
"sv('m',v4);}\n";

        

          
prog_uchar js4[] PROGMEM =
"function w(s){document.writeln(s)}\n"
"function sd(l,v) {\n"
"if(document.getElementById(l)) document.getElementById(l).disabled=v;}\n"
"function di(){\n"
"var v=true;\n"
"if (document.getElementById('a').checked) v=false;\n"
"sd('h',v);\n"
"sd('u',v);\n"
"sd('p',v);}\n"
"function wi(l) {\n"
"w(\"<input type=text name=\"+l+\" id=\"+l+\" value='' size=15 /><br />\");}\n"
"function sv(l,v) {\n"
"document.getElementById(l).value=v;}\n"
"function sc(l,v) {\n"
"document.getElementById(l).checked=v;}\n"

;


byte *htmlWebpage_js[] = {js0,
                          js1,
                          js2,
                          js3,
                          js4};

static void homePage(BufferFiller& buf) {

    bfill.emit_p(PSTR("$F\r\n$F"
"<div data-role='page'>"
	"<div data-role='header'>"
		"<h1>Main page</h1>"
	"</div>"
"<br><ul data-role='listview' data-inset='true'>"
        "<li><a href='r' data-ajax='false'>Remote</a></li>"
        "<li><a href='s0' data-ajax='false'>Schedule</a></li>"
        "<li><a href='c' data-ajax='false'>Network</a></li>"
        "<li><a href='n' data-ajax='false'>NTP</a></li>"
        "<li><a href='d' data-ajax='false'>DynDNS</a></li>"
        "<li><a href='h' data-ajax='false'>HTTP options</a></li>"
"</ul>"        
        
        ), okHeader, loadConfig(CONFIG_HTTPJQENABLE) ? jqHTML : njqHTML, airConroller_getMode());


  //  buf.emit_p(PSTR("<br>FW $D.$D<br>"), FW_VER,FW_SUBVER);
#if DEBUG
//    buf.emit_p(PSTR("$D bytes free RAM <br>"), freeRam());    
#endif

    byte  d = ((millis() / 1000) / 86400L ); //60*60*24
    byte  h = ((millis() / 1000) / 3600) % 24;
    byte  m = ((millis() / 1000) / 60) % 60;
    buf.emit_p(PSTR("Up: $D$D days $D$Dh $D$Dm<br>"), d/10, d%10, h/10, h%10, m/10, m%10);
    
//    buf.emit_p(PSTR("NTP time "));
//    if(timeStatus()==timeNotSet) buf.emit_p(PSTR("<b>not</b> "));
//    buf.emit_p(PSTR("set: $D/$D/$D $D:$D<br>"), year(), month(), day(), hour(),minute() );
        
    buf.emit_p(PSTR("WAN IP: $D.$D.$D.$D<br>"), extip[0], extip[1], extip[2], extip[3]);
//    buf.emit_p(PSTR("Client IP: $D.$D.$D.$D"), ether.buffer[0x1A],ether.buffer[0x1B],ether.buffer[0x1C],ether.buffer[0x1D]);
    buf.emit_p(PSTR("</body></html>"));

}

static int getIntArg(const char* data, const char* key, int value =-1) {
    char temp[10];
    memset(temp,NULL,sizeof(temp));
    if (ether.findKeyVal(data + 7, temp, sizeof temp, key) > 0)
        value = atoi(temp);
    return value;
}

static char* getArg(const char* data, const char* key) {
    char temp[64];
    memset(temp,NULL,sizeof(temp));
    ether.findKeyVal(data + 7, temp, sizeof temp, key); 
    return temp;
}

static void configPage(const char* data, BufferFiller& buf) {
    // pick up submitted data, if present
    if (data[6] == '?') {

    
//        ether.parseIp(extip, (char*) Ethernet::buffer + off);
//  Serial.print((const char*) Ethernet::buffer + off);


        byte ip[4];
        if(ether.parseIp(ip,getArg(data, "i"))==0)
            saveConfig2(ip,CONFIG_IP,4);
        if(ether.parseIp(ip,getArg(data, "g"))==0)        
            saveConfig2(ip,CONFIG_GW,4);
        if(ether.parseIp(ip,getArg(data, "d"))==0)                
            saveConfig2(ip,CONFIG_DNS,4);
           
     bfill.emit_p(rdHeader);
//     bfill.emit_p(rdHTML, 2,"c"," ",2 );
         
            return;
        
    }
    // else show a configuration form
    bfill.emit_p(PSTR("$F\r\n$F"
"<div data-role='page'>"
	"<div data-role='header'>"
		"<h1>Network Settings</h1>"
	"</div>"
"<form data-ajax='false'>"
"<script src='p4.js'></script>"
"<script type='text/javascript'>"
"w('IP:');\n"
"wi('i');\n"
"w('GW:');\n"
"wi('g');\n"
"w('DNS:');\n"
"wi('d');\n"
"sv('i','$D.$D.$D.$D');\n"
"sv('g','$D.$D.$D.$D');\n"
"sv('d','$D.$D.$D.$D');\n"
"</script>"
"<script src='p1.js'></script>"
        "</form>"
        
        "</div>"
"</body>"
"</html>"
        ),okHeader,loadConfig(CONFIG_HTTPJQENABLE) ? jqHTML : njqHTML,
        loadConfig(CONFIG_IP),loadConfig(CONFIG_IP+1),loadConfig(CONFIG_IP+2),loadConfig(CONFIG_IP+3),
        loadConfig(CONFIG_GW),loadConfig(CONFIG_GW+1),loadConfig(CONFIG_GW+2),loadConfig(CONFIG_GW+3),
        loadConfig(CONFIG_DNS),loadConfig(CONFIG_DNS+1),loadConfig(CONFIG_DNS+2),loadConfig(CONFIG_DNS+3));
}


static void RCPage(const char* data, BufferFiller& buf) {
    // pick up submitted data, if present
    if (data[6] == '?') {
      
//    char* temp;

   int temp = getIntArg(data, "s");           //state
     
    switch (temp) {
    case 0:
             airController_off(); 
      break;
    case 1:
             airController_on(); 
      break;
    default: 
      break;
  }

  
    temp=getIntArg(data, "t");     //temp
    if(temp) { 
       if(temp>=14 && temp <=32) {
         airController_setTemp(temp);
//         Serial.println(temp);
       }
     }
    
    temp=getIntArg(data, "f");   //fan
    if (temp) {
        if(temp >=1 && temp <=250) {
        airController_setFan(temp);
  //           Serial.println(temp);        
      }
    }

      temp=getIntArg(data, "d",4);   ///mode
        if(temp >=0 && temp <=7) {
        airController_setMode(temp);
    }


      temp=getIntArg(data, "a");   //aux
      if(temp==0 || temp==1 || temp==16) airController_setAux(temp); 
            
      airController_checksum();  
      irsend.sendDaikin(daikin, 8,0); 
      delay(29);
      irsend.sendDaikin(daikin, 19,8); 

      
            // redirect to the home page
            bfill.emit_p(rdHeader);
            return;
        
    }
    // else show a configuration form
    bfill.emit_p(PSTR("$F\r\n$F"
"<div data-role='page'>"
	"<div data-role='header'>"
		"<h1>Remote control</h1>"
	"</div>"
"<form data-ajax='false'>"
        "<script src=p0.js></script>"
        "<script src=p1.js></script>"
"</form>"
          "<script src=p3.js></script>"
          "<script type='text/javascript'>"
          "svi($D,$D,$D,$D,$D);"
          "</script>"
"</div>"
"</body>"
"</html>"

),okHeader, loadConfig(CONFIG_HTTPJQENABLE) ? jqHTML : njqHTML, airConroller_getMode(), airConroller_getTemp(),airConroller_getFan(),airController_getAux(), airConroller_getState() );
}


static void SchedPage(const char* data, BufferFiller& buf) {
    // pick up submitted data, if present
    if (data[7] == '?') {

      
     irstate.mode=getIntArg(data, "d",0);  //mode
     irstate.temp=getIntArg(data, "t",15);  //temp
     irstate.fan=getIntArg(data, "f");   //fan
     irstate.aux=getIntArg(data, "a");  //aux
     irstate.state=getIntArg(data, "s");  //state
     irstate.enabled=getIntArg(data, "e");  //enabled
     irstate.sched=getIntArg(data, "c");  //sched
     irstate.hour=getIntArg(data, "h");  //hour
     irstate.minutes=getIntArg(data, "m");  //min
     
     
//     irstate.lastused=now();
     int page=data[6]-'0';
     if(page<0 || page>9) return;
     
     saveIRstate(page);     

     bfill.emit_p(rdHeader);
//     bfill.emit_p(rdHTML, TIME_REDIRECT_DELAY, "s",page+"0",TIME_REDIRECT_DELAY );
     return;
    }

    //load page's configuration from EEPROM    
    loadIRstate(data[6]-'0');
    
    // else show a configuration form
    bfill.emit_p(PSTR("$F\r\n$F"
        "<div data-role='page'>"
	"<div data-role='header'>"
	"<h1>Timer&nbsp;&nbsp;$D</h1>"
	"</div>"),okHeader,loadConfig(CONFIG_HTTPJQENABLE) ? jqHTML : njqHTML, data[6]-'0');

    if(timeStatus()==timeNotSet) 
       buf.emit_p(PSTR("NTP time <b>not set!</b> ")); 

     bfill.emit_p(PSTR("<br><form data-ajax='false'>"
      "<script src=p2.js></script>"
      "<script src=p0.js></script>"
      "<script src=p1.js></script>"
        "</form>"
          "<script src=p3.js></script>"
          "<script type='text/javascript'>"
          "svt($D,$D,$D,$D);"
          "svi($D,$D,$D,$D,$D);"
          "</script>"
   "</div>"
"</body>"
"</html>"),irstate.enabled, irstate.sched,irstate.hour,irstate.minutes,irstate.mode, irstate.temp,irstate.fan,irstate.aux,irstate.state );
}



static void JSONPage(const char* data, BufferFiller& buf) {

    bfill.emit_p(PSTR("$F\r\n"
        "{\"state\":\"$D\",\"mode\":\"$D\",\"temp\":\"$D\",\"fan\":\"$D\",\"aux\":\"$D\"}"),okHeader, airConroller_getState(), airConroller_getMode(), airConroller_getTemp(),airConroller_getFan(),airController_getAux());
}

// print javascript webpage
static void JSPage(const char* data, BufferFiller& buf, byte idx) {

  bfill.emit_p(PSTR("$F$F"), okHeaderjs, htmlWebpage_js[idx]);
}

static void DDNScallbackupd (byte status, word off, word len) {
  #if SERIAL
  writestrln(PSTR("Got DDNS feedback:"));
  Serial.println((char*)&Ethernet::buffer[off]);  
  #endif
}

// called when the client request is complete
static void DDNScallback (byte status, word off, word len) {
 // Ethernet::buffer[off+300] = 0;
 // ether.parseIp(extip, (char*) Ethernet::buffer + off);
//  Serial.print((const char*) Ethernet::buffer + off);
//  Serial.println("...");

const char headerEnd[2] = {'\r','\n' };
  int contentLen = 0;

  if (off != 0)
  {
    uint16_t pos = off;
    while ( Ethernet::buffer[pos] )    // loop until end of buffer (or we break out having found what we wanted)
    {
      // Look for line with \r\n on its own
      if( strncmp ((char*)&Ethernet::buffer[pos],headerEnd, 2) == 0 ) {
        pos += 2;
        break;
      }
     
      
      // Scan to end of line
      while( Ethernet::buffer[pos++] != '\r' ) { }
      while( Ethernet::buffer[pos++] != '\n' ) { }
      
 
    } 
    
    // search for our IP in the http 
      uint16_t startpos = pos;
      uint16_t endpos = pos;
      
      boolean eop = false; 
      
      while((Ethernet::buffer[pos]<'.' && Ethernet::buffer[pos] >'9') || Ethernet::buffer[pos]!='\\')
      {
        pos++;
      }
      
      
      while(!eop){
        switch(Ethernet::buffer[pos]){
          case '0':
          case '1':
          case '2':
          case '3':
          case '4':
          case '5':
          case '6':
          case '7':
          case '8':
          case '9':
          case '.':
            eop = false;
            break;
          default:
           eop=true;
           break;
        }
        pos++;
      }
      endpos = pos-1;
      Ethernet::buffer[endpos] = '\0';
      //Serial.println((char*)&Ethernet::buffer[startpos]);
      if(ether.parseIp(extip, (char*)&Ethernet::buffer[startpos])==0) {
        byte currwan[4];
        loadConfig2(currwan,CONFIG_WANIP,4); 
        // check if WAN is not zeroes
        if(memcmp(currwan,extip,4)!=0) {
            //update DynDNS
             if(loadConfig(CONFIG_DDNSENABLE)==1) {
                saveConfig2(extip,CONFIG_WANIP,4);
                 updateddns();

  } //DDNS updates enabled?
  } // WAN changed?
  } // WAN IP parses?
  } //site responded? 
  
  
}

static void NTPPage(const char* data, BufferFiller& buf) {
    // pick up submitted data, if present
    if (data[6] == '?') {

 
        saveConfig2((byte*)getArg(data, "n"),CONFIG_NTP,31);
        int tof=getIntArg(data, "t");       
        saveConfig(CONFIG_TIMEZONEOFFSET,(byte)tof);

     bfill.emit_p(rdHeader);
//     bfill.emit_p(rdHTML, TIME_REDIRECT_DELAY,"n", " ",TIME_REDIRECT_DELAY );
     return;

    }

    // else show a configuration form

    bfill.emit_p(PSTR("$F\r\n$F"
"<div data-role='page'>"
	"<div data-role='header'>"
		"<h1>NTP Settings</h1>"
	"</div>"
),okHeader,loadConfig(CONFIG_HTTPJQENABLE) ? jqHTML : njqHTML);

    buf.emit_p(PSTR("NTP time "));
    if(timeStatus()==timeNotSet) 
       buf.emit_p(PSTR("<b>not set!</b> ")); 
    else
       buf.emit_p(PSTR("set: $D/$D/$D $D:$D"), year(), month(), day(), hour(),minute() );


    bfill.emit_p(PSTR("<form data-ajax='false'>"
"<br>NTP Server: <input type='text' size=32 name=n id=n value='$E'>"      
"<br>Time offset: <input type='text' size=2 name=t id=t value='$D'>"    
"<br><script src='p1.js'></script>"
        "</form>"        
        "</div>"
"</body>"
"</html>"
        ), CONFIG_EEPROM_ADDR+CONFIG_NTP ,(int8_t)loadConfig(CONFIG_TIMEZONEOFFSET) ); //CONFIG_EEPROM_ADDR+CONFIG_NTP
                 
}

static void HTTPPage(const char* data, BufferFiller& buf) {
    // pick up submitted data, if present
    if (data[6] == '?') {

        byte temp=*getArg(data, "q");
        if(temp!=0) saveConfig(CONFIG_HTTPJQENABLE,1); else saveConfig(CONFIG_HTTPJQENABLE,0);
        temp=*getArg(data, "a");
        if(temp!=0) {
          saveConfig(CONFIG_HTTPAUTH,1); 
          saveConfig2((byte*)getArg(data, "u"),CONFIG_HTTPUSER,10);
          saveConfig2((byte*)getArg(data, "p"),CONFIG_HTTPPASS,10); 
        }
        else saveConfig(CONFIG_HTTPAUTH,0);
        
        uint16_t port=getIntArg(data, "t",80);       
        saveConfig(CONFIG_HTTPPORT,(byte)port);
        saveConfig(CONFIG_HTTPPORT+1,(byte)(port >>8));

     bfill.emit_p(rdHeader);
//     bfill.emit_p(rdHTML, TIME_REDIRECT_DELAY,"h", " ",TIME_REDIRECT_DELAY );
     return;

    }

    // else show a configuration form
    
    uint16_t port = loadConfig(CONFIG_HTTPPORT+1)<<8;
    port+=loadConfig(CONFIG_HTTPPORT);
    
    bfill.emit_p(PSTR("$F\r\n$F"
"<div data-role='page'>"
	"<div data-role='header'>"
		"<h1>HTTP Options</h1>"
	"</div>"
"<form data-ajax='false'>"
"<br>Server port: <input type='text' name=t id=t value='$D'>"    
"<br><input type='checkbox' name=q id=q /><label for=q>JQ UI?</label>"
"<br><input type='checkbox' name=a id=a onclick='di()'/><label for=a>HTTP Auth?</label>"
"<br>User: <input type='text' name=u id=u value='$E'>"      
"<br>Pass: <input type='text' name=p id=p value='$E'><br>"      
"<script src='p4.js'></script>"
"<script src='p1.js'></script>"
        ),okHeader,loadConfig(CONFIG_HTTPJQENABLE) ? jqHTML : njqHTML, port,CONFIG_EEPROM_ADDR+CONFIG_HTTPUSER,CONFIG_EEPROM_ADDR+CONFIG_HTTPPASS ); //CONFIG_EEPROM_ADDR+CONFIG_NTP

    bfill.emit_p(PSTR("<script type='text/javascript'>"
    "sc('q',$D);\n"
    "sc('a',$D);\n"
    "di();\n"
    "</script>"
    "</form></div></body></html>"
    ),loadConfig(CONFIG_HTTPJQENABLE),loadConfig(CONFIG_HTTPAUTH));
    
                 
}





static void DDNSPage(const char* data, BufferFiller& buf) {
    // pick up submitted data, if present
    if (data[6] == '?') {

        byte temp=*getArg(data, "a");
        if(temp!=0) {
          saveConfig(CONFIG_DDNSENABLE,1); 
          saveConfig2((byte*)getArg(data, "h"),CONFIG_DDNSHOST,32);
          saveConfig2((byte*)getArg(data, "u"),CONFIG_DDNSUSER,10);
          saveConfig2((byte*)getArg(data, "p"),CONFIG_DDNSPASS,10);                   
        }
        else saveConfig(CONFIG_DDNSENABLE,0);
        
     bfill.emit_p(rdHeader);
//     bfill.emit_p(rdHTML, TIME_REDIRECT_DELAY,"d", " ",TIME_REDIRECT_DELAY );
     return;

    }

    // else show a configuration form
    
    bfill.emit_p(PSTR("$F\r\n$F"
"<div data-role='page'>"
	"<div data-role='header'>"
		"<h1>DDNS Options</h1>"
	"</div>"
"<form data-ajax='false'>"
"<br><input type='checkbox' name=a id=a onclick='di()'/><label for=a>DynDNS enabled?</label>"
"<br>Host: <input type='text' name=h id=h value='$E'>"      
"<br>User: <input type='text' name=u id=u value='$E'>"      
"<br>Pass: <input type='text' name=p id=p value='$E'><br><input type='hidden' name=z value='0'></input>"      
"<script src='p4.js'></script>"
"<script src='p1.js'></script>"
        ),okHeader,loadConfig(CONFIG_HTTPJQENABLE) ? jqHTML : njqHTML,CONFIG_EEPROM_ADDR+CONFIG_DDNSHOST,CONFIG_EEPROM_ADDR+CONFIG_DDNSUSER,CONFIG_EEPROM_ADDR+CONFIG_DDNSPASS ); //CONFIG_EEPROM_ADDR+CONFIG_NTP

    bfill.emit_p(PSTR("<script type='text/javascript'>"
    "sc('a',$D);\n"
    "di();\n"
    "</script>"
    "</form></div></body></html>"
    ),loadConfig(CONFIG_DDNSENABLE));
    
                 
}






void setup(){

    // set the digital pin as output:
  pinMode(ledPin, OUTPUT);     
  pinMode(switchPin, INPUT);    // Set the switch pin as input
  
#if SERIAL
    Serial.begin(9600);
    writestrln(PSTR("\n[Configuration]"));
#endif


  if ((eeprom_read_byte(CONFIG_EEPROM_ADDR + CONFIG_VALID)!=0xAA) || (digitalRead(switchPin) == HIGH)) {
  //Load defaults and save to EEPROM

    reseteeprom();  
  }

    const byte mymac[] = { 0x74,0x69,0x69,0x2D,0x30,0x31 };

    if (ether.begin(sizeof Ethernet::buffer, mymac) == 0) {
      #if SERIAL
            writestrln(PSTR("Failed to start ethernet.."));
      #endif
    }
     
    byte ip[4];
    byte gw[4];  
    byte dns[4];
    loadConfig2(ip,CONFIG_IP,4);
    loadConfig2(gw,CONFIG_GW,4);
    loadConfig2(dns,CONFIG_DNS,4);
    
    if (!ether.staticSetup(ip,gw,dns)) {
      #if SERIAL
            writestrln(PSTR("Failed static IP address.."));
      #endif     
    }
    
#if SERIAL
    writestr(PSTR("IP:"));
    printIp(ether.myip);
//    ether.printIp("Netmask: ", ether.mymask);
    writestr(PSTR("GW:"));
    printIp(ether.gwip);
    writestr(PSTR("DNS:"));    
    printIp(ether.dnsip);     
    writestr(PSTR("MAC:"));
    printMAC(ether.mymac);
    Serial.println();
    writestr(PSTR("Free RAM: "));
    Serial.println(freeRam());    
#endif

    ether.registerPingCallback(gotPinged);

   memset(ntp,0,4);
   getntpip();
   setSyncInterval(3600*3);
   setSyncProvider(getNtpTime);


   previousMillisExtIP=millis();
   if (ether.dnsLookup(DDNSchecksite)) {
       ether.browseUrl(PSTR("/"), "","", DDNSchecksite, DDNScallback);
   } 


    ether.hisport = loadConfig(CONFIG_HTTPPORT);

#if SERIAL
    printHelp();
#endif



}


void loop(){
  

//  if (ether.dhcpExpired())
//       ether.dhcpSetup();

if(digitalRead(switchPin) == HIGH){
    digitalWrite(ledPin, LOW);            
            airController_checksum();
            irsend.sendDaikin(daikin, 8,0); 
            delay(29);
            irsend.sendDaikin(daikin, 19,8); 
            delay(500);
    digitalWrite(ledPin, HIGH);            
}


    word len = ether.packetReceive();
    word pos = ether.packetLoop(len);

    // check if valid tcp data is received
    if (pos) {
        bfill = ether.tcpOffset();
        char* data = (char *) Ethernet::buffer + pos;
#if SERIAL
       // Serial.println(data);
       // Serial.println(freeRam());
#endif
        // receive buf hasn't been clobbered by reply yet


    byte authorized=false;
    
    if(loadConfig(CONFIG_HTTPAUTH)==0) {
      authorized=true;
      authorizedT=millis();
    }
    
    //If the same IP as the last authorized and time since last auth from that IP is < authorizationInterval then consider the user authorized
    if(authorizedIP[0]==ether.buffer[0x1A] && authorizedIP[1]==ether.buffer[0x1B] && authorizedIP[2]==ether.buffer[0x1C] && authorizedIP[3]==ether.buffer[0x1D] && millis()-authorizedT<authorizationInterval ){
      authorized=true;
      authorizedT=millis();
    }

    if(getIntArg(data, "z",-1)==7) authorized=true;

    int pos2=pos;    
    while ( Ethernet::buffer[pos2++] )    // loop until end of buffer (or we break out having found what we wanted)
    {
      // Look for Authorization header along with the hardcoded password
      if( strncmp_P ((char*)&Ethernet::buffer[pos2],PSTR("Authorization: Basic "), 21) == 0 ) {   

              
        if(verifycred(pos2+21))  {
         authorized=true;
          authorizedIP[0]= ether.buffer[0x1A];   //If authorized, save the last IP
          authorizedIP[1]= ether.buffer[0x1B];
          authorizedIP[2]= ether.buffer[0x1C];
          authorizedIP[3]= ether.buffer[0x1D];
          authorizedT=millis();                  //Save time when last authorized, so we don't bother the user with authorization the next "authorizationInterval" millis
          break;
      }
      }
    }
    
    if(!authorized)  {    
            bfill.emit_p(PSTR(
                "HTTP/1.0 401 Unauthorized\r\n"
                "Content-Type: text/html\r\n"
                "WWW-Authenticate: Basic realm=\"Remote Controller\""
                "\r\n"
                "<h1>401 Unauthorized</h1>"));  
        ether.httpServerReply(bfill.position()); // send web page data
    }

  else {
        if (strncmp_P(data,PSTR("GET / "), 6) == 0)
            homePage(bfill);
        else if (strncmp_P(data,PSTR("GET /c"), 6) == 0)
            configPage(data, bfill);
        else if (strncmp_P(data,PSTR("GET /r"), 6) == 0)
            RCPage(data, bfill);
        else if (strncmp_P(data,PSTR("GET /s"), 6) == 0)
            SchedPage(data, bfill);
        else if (strncmp_P(data,PSTR("GET /n"), 6) == 0)
            NTPPage(data, bfill);
        else if (strncmp_P(data,PSTR("GET /h"), 6) == 0)
            HTTPPage(data, bfill);
        else if (strncmp_P(data,PSTR("GET /d"), 6) == 0)
            DDNSPage(data, bfill);
        else if (strncmp_P(data,PSTR("GET /j"), 6) == 0)
            JSONPage(data, bfill);         
        else if (strncmp_P(data,PSTR("GET /y"), 6) == 0)
        {
            bfill.emit_p(PSTR("HTTP/1.0 200 OK\r\n\r\n OK"));
            ether.httpServerReply(bfill.position()); // send web page data
            restartac();         
        }
        else if (strncmp_P(data,PSTR("GET /x"), 6) == 0)
            {
              bfill.emit_p(rdHeader);
              ether.httpServerReply(bfill.position()); // send web page data
              delay(3000);
              reboot();
            }
        else if (strncmp_P(data,PSTR("GET /p"), 6) == 0)
            JSPage(data, bfill,data[6]-'0');            
        else
            bfill.emit_p(PSTR(
                "HTTP/1.0 401 Unauthorized\r\n"
                "Content-Type: text/html\r\n"
                "\r\n"
                "<h1>401 Unauthorized</h1>"));  
        ether.httpServerReply(bfill.position()); // send web page data
    }
 }



#if SERIAL

          if (Serial.available()) {
	        switch (Serial.read()) {
			case '1':
		     	  writestrln(PSTR("Initializing EEPROM.."));
                          reseteeprom();
			  break;
			case '2':
		     	  writestrln(PSTR("Updating DDNS.."));
                          updateddns();
			  break;
			case 'h':
                          printHelp();
			  break;
			default:
			  break;
				}
          }


#endif

if (millis()- previousMillisTimer > 30*1000 ){   //every 30 sec
if(timeStatus()!=timeNotSet)
           handletimer();
}

if (millis()- previousMillisExtIP > intervalextip ){ 
  previousMillisExtIP=millis();
     if (ether.dnsLookup(DDNSchecksite))
          ether.browseUrl(PSTR("/"), "","", DDNSchecksite, DDNScallback);
}


//  unsigned long currentMillis = millis();
 
  if(millis() - previousMillis > interval) {
    // save the last time you blinked the LED 
    previousMillis = millis();   

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW)
      ledState = HIGH;
    else
      ledState = LOW;

    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
  }
  
}




byte verifycred(int pos){
  
      byte temp[20];
      char temp2[32];

      loadConfig2(temp,CONFIG_HTTPUSER,10);  //load the username

      int i;
      for(i=0;i<9;i++) {
        if(temp[i]=='\0') break;            //find the end and insert a ":" 
      }
      temp[i]=':';
      i++;
      loadConfig2(temp+i,CONFIG_HTTPPASS,10);    //load the password
      for(;i<20;i++) {
        if(temp[i]=='\0') break;
      }

      base64_encode(temp2,32,(char *)temp,i);    //base64 encode the user:pass string


      for(i=0;i<32;i++) {
        if(temp2[i]=='\0') break;            //find the end 
      }


      if(memcmp(temp2,(char*)&Ethernet::buffer[pos],i)==0) 
         return true;
      else 
         return false;
}

void handletimer(){

 if(timeStatus()!=timeNotSet) {
      
   for (int i=0;i<numtimers;i++) {
      loadIRstate(i);
      if(irstate.enabled==1) {
        
     //1=SUN;2=MON...7=SAT
      byte d=weekday(now());
      //1=MON;2=TUE...7=SUN
      if(d==1) d=7; else d--;
      //d2=mask 
      byte d2=0x01<<(d-1);     
       
      if((irstate.sched & d2)>>(d-1)) {
        if(hour(now())==irstate.hour) {
          if(minute(now())==irstate.minutes) {
            //timer kick-in

            if(irstate.state==0) airController_off(); 
            if(irstate.state==1) airController_on(); 
            airController_setAux(irstate.aux); 
            airController_setTemp(irstate.temp);
            airController_setFan(irstate.fan);
            airController_setMode(irstate.mode);
            
            irsend.sendDaikin(daikin, 8,0); 
            delay(29);
            irsend.sendDaikin(daikin, 19,8); 
            
          }
        }
      }
      }     
   }
}

previousMillisTimer=millis();
}


// =============
// NTP Functions
// =============

unsigned long ntp_wait_response()
{
  uint32_t time;
  for (uint32_t i=0; i<100000; i++) {
    ether.packetLoop(ether.packetReceive());
    if (ether.ntpProcessAnswer(&time, ntpclientportL))
    {
      if ((time & 0x80000000UL) ==0){
        time+=2085978496;
      }else{
        time-=2208988800UL;
      }   
      return time + (int32_t)3600*(int32_t)((int8_t)loadConfig(CONFIG_TIMEZONEOFFSET));
    }
    
  }  
  return 0;
}

unsigned long getNtpTime()
{
  unsigned long ans;

  if(ntp[0]==0&&ntp[1]==0&&ntp[2]==0&&ntp[3]==0) getntpip();
  
  if(!(ntp[0]==0&&ntp[1]==0&&ntp[2]==0&&ntp[3]==0)){
  byte tick = 0;
  do {
    ether.ntpRequest((byte *)ntp, ntpclientportL);
    ans = ntp_wait_response();
    delay(250);
    tick ++;
  } while( ans == 0 && tick < 15 );  
  }
  return ans;
}



uint8_t airController_checksum()
{
	uint8_t sum = 0;
	uint8_t i;


	for(i = 0; i <= 6; i++){
		sum += daikin[i];
	}

        daikin[7] = sum &0xFF;
        
        sum=0;
	for(i = 8; i <= 25; i++){
		sum += daikin[i];
        }

        daikin[26] = sum &0xFF;

        
}



void airController_on(){
	//state = ON;
	daikin[13] |= 0x01;
	airController_checksum();
}

void airController_off(){
	//state = OFF;
	daikin[13] &= 0xFE;
	airController_checksum();
}

void airController_setAux(uint8_t aux){
	daikin[21] = aux;
	airController_checksum();
}

uint8_t airController_getAux(){
	return daikin[21];
}


void airController_setTemp(uint8_t temp)
{
	daikin[14] = (temp)*2;
	airController_checksum();
}


void airController_setFan(uint8_t fan)
{
	daikin[16] = fan;
	airController_checksum();
}


uint8_t airConroller_getTemp()
{
	return (daikin[14])/2;
}


uint8_t airConroller_getMode()
{

/*
Modes: b6+b5+b4
011 = Cool
100 = Heat (temp 23)
110 = FAN (temp not shown, but 25)
000 = Fully Automatic (temp 25)
010 = DRY (temp 0xc0 = 96 degrees c)
*/

	return (daikin[13])>>4;

}


void airController_setMode(uint8_t mode)
{
	daikin[13]=mode<<4 | airConroller_getState();
	airController_checksum();
}


uint8_t airConroller_getState()
{
	return (daikin[13])&0x01;
}

uint8_t airConroller_getFan()
{
	return (daikin[16]);
}


// A dirty hack to jump to start of boot loader
void reboot() {
    asm volatile (" jmp 0x7C00");
}

void restartac () {
  
            if(airConroller_getState()==1) {
              
              airController_off(); 
              irstate.aux=airController_getAux();
              irstate.temp=airConroller_getTemp();
              irstate.fan= airConroller_getFan();
              irstate.mode=airConroller_getMode();
             
              irsend.sendDaikin(daikin, 8,0); 
              delay(29);
              irsend.sendDaikin(daikin, 19,8); 
              
              delay (10000);
              
              airController_on(); 
              airController_setAux(0); 
              airController_setTemp(irstate.temp);
              airController_setFan(irstate.fan);
              airController_setMode(irstate.mode);
             
              irsend.sendDaikin(daikin, 8,0); 
              delay(29);
              irsend.sendDaikin(daikin, 19,8); 

            }
}


/*

if(memcmp_P(myarray, array1, 5) == 0)
  {
    Serial.println("YAY!");
  }
  
*/


#if SERIAL

static void writestr(PGM_P str)
{
  uint8_t c;

  while ((c=pgm_read_byte(str)) != 0) {
    Serial.print(c);
    str++;
  }
}

static void writestrln(PGM_P str)
{
  writestr(str);
  Serial.println();
}


void printIp (const byte *buf) {
    for (byte i = 0; i < 4; ++i) {
        Serial.print( buf[i], DEC );
        if (i < 3)
            Serial.print('.');
    }
    Serial.println();
  }
  
void printMAC( uint8_t *buf ) {
  for( int i = 0; i < 6; i++ ) {
    Serial.print( buf[i], HEX );
    if( i<5 )
      Serial.print( ":" );
  }
}

    
#endif
 
int base64_decode( char *out, const char *in, int out_size)
{
    int i, v;
    char *dst = out;
        int index;

    v = 0;
    for (i = 0; in[i] && in[i] != '='; i++)
        {
                index = in[i]-43;

                if ( pgm_read_byte( &map64[index] ) == 0xff)
                return -1;
        
                v = (v << 6) + pgm_read_byte( &map64[index] );
        
                if (i & 3)
                {
            if (dst - out < out_size)
                        {
                *dst++ = v >> (6 - 2 * (i & 3));
            }
        }
        }
 
    return( dst - out );
}


char * base64_encode(char *out, int out_size, const char *in, int in_size)
{
        char *ret, *dst;
    unsigned long i_bits = 0;
    int i_shift = 0;
    int bytes_remaining = in_size;
 
    if ( in_size >= 65535 / 4 || out_size < ( in_size+2 ) / 3 * 4 + 1 )
        return NULL;

    ret = dst = out;
    while (bytes_remaining)
        {
        i_bits = (i_bits << 8) + *in++;
        bytes_remaining--;
        i_shift += 8;

        do
                {
            *dst++ = pgm_read_byte( &b64[ (i_bits << 6 >> i_shift) & 0x3f] ) ;
            i_shift -= 6;
        }
                while (i_shift > 6 || (bytes_remaining == 0 && i_shift > 0));
        }

        while ((dst - ret) & 3)
        *dst++ = '=';

        *dst = '\0';

    return ret;
}

void getntpip() {
      byte ntp2[32];// ={'n','t','p','.','y','o','u','r','.','o','r','g',0x00};
//    saveConfig2(ntp2,CONFIG_NTP,13);
   
    loadConfig2(ntp2,CONFIG_NTP,31);    
    if(ether.parseIp(ntp,(char *)ntp2)==0){
#if SERIAL
      writestr(PSTR("NTP is IP:"));      
      printIp(ntp);
#endif      
    }
     else {
       memset(ntp,0,4);
     loadConfig2(ntp2,CONFIG_NTP,31);   
     if(ether.dnsLookup2((char *)ntp2)) {
      ether.copyIp(ntp,ether.hisip);
#if SERIAL  
      writestr(PSTR("NTP is name:"));  
      Serial.println((char *)ntp2);
      printIp(ntp);
#endif      
    } else       
    {
     #if SERIAL

      writestrln(PSTR("NTP doesn't resolve"));  
      
      #endif
    }
     }
}

#if SERIAL

static void printHelp(void)
{
  writestrln(PSTR(\
"\n\nPress a key:\n" \
"  1: Initialize EEPROM \n" \
"  2: Update DDNS now\n" \
"  H: Print this help menu" \
         ));
}
   
#endif

   char * ProgmemString (prog_char * text)
    {
    byte x = 0;
    static char buffer [40];
  
    do
      {
      buffer [x] = pgm_read_byte_near (text + x);
      } while (buffer [x++] != 0);
    return buffer;
    }
    

byte updateddns() {
                if (ether.dnsLookup(DDNSupdatesite)) {
                         
                  static char temp[48];
                  static char temp2[48];
                  

                  loadConfig2((byte *)temp,CONFIG_DDNSUSER,10);  //load the username
                  int i=0;
                  for(;i<9;i++) {
                    if(temp[i]=='\0') break;            //find the end and insert a ":" 
                  }
                  temp[i]=':';
                  i++;
                  loadConfig2((byte *)temp+i,CONFIG_DDNSPASS,10);    //load the password
                  for(;i<21;i++) {
                    if(temp[i]=='\0') break;
                  }
                  
                  base64_encode(temp2,32,(char *)temp,i);    //base64 encode the user:pass string

                  for(i=0;i<32;i++) {
                    if(temp2[i]=='\0') break;            //find the end 
                  }
             //     temp2[i++]='\r';
             //     temp2[i++]='\n';
             //     temp2[i++]='\0';
                  
//                   memcpy(temp,ProgmemString(PSTR("Authorization: Basic ")),21);
                   memcpy(temp,temp2,i);
//                   memcpy(&temp[i],ProgmemString(PSTR("\r\n\0")),4);  //include the terminating 0                                       
//                   memcpy(temp2,ProgmemString(PSTR("?hostname=")),10);
                   loadConfig2((byte *)&temp2,CONFIG_DDNSHOST,32);
                   

                                       
                   i=1;   
                   for(;i++;i<32){if(temp2[i]==0) break;}
                   memcpy(&temp2[i],ProgmemString(PSTR("&myip=")),6);
                   ether.makeNetStr(&temp2[i+6],extip,4,'.',10);
                   
#ifdef DEBUG        
//                    printIp(ether.hisip);
//                    Serial.println(temp);
//                    Serial.println(temp2);
#endif
                    ether.browseUrl(PSTR("/nic/update?hostname="), temp2, temp, DDNSupdatesite2, DDNScallbackupd);
//                    ether.browseUrl(PSTR("/~hariabql/headers.php/"), temp2, (char *)temp, DDNSupdatesite, DDNScallbackupd);
                    
                    return(1);
                    
     }  // Could resolve dyndns update site
      else return (0);
     
     }
     
     void reseteeprom() {
       
      for(int i=0;i<20;i++){
    digitalWrite(ledPin, HIGH);
    delay(50);
    digitalWrite(ledPin, LOW);
    delay(50);
}    
    saveConfig(CONFIG_USEDHCP,0);
    
    saveConfig(CONFIG_IP,192);
    saveConfig(CONFIG_IP+1,168);
    saveConfig(CONFIG_IP+2,1);
    saveConfig(CONFIG_IP+3,8);    
    
    saveConfig(CONFIG_GW,192);
    saveConfig(CONFIG_GW+1,168);
    saveConfig(CONFIG_GW+2,1);
    saveConfig(CONFIG_GW+3,1);    
  
    saveConfig(CONFIG_DNS,8);
    saveConfig(CONFIG_DNS+1,8);
    saveConfig(CONFIG_DNS+2,8);
    saveConfig(CONFIG_DNS+3,8);    
    
    byte tmp[32]={'n','t','p','.','y','o','u','r','.','o','r','g',0x00};
    saveConfig2(tmp,CONFIG_NTP,13);    

    saveConfig(CONFIG_TIMEZONEOFFSET,2);
    saveConfig(CONFIG_HTTPPORT,80);
    saveConfig(CONFIG_HTTPPORT+1,0);
    saveConfig(CONFIG_HTTPAUTH,0);
    saveConfig(CONFIG_HTTPJQENABLE,0);
    saveConfig(CONFIG_DDNSENABLE,0);   
    saveConfig(CONFIG_WANIP,0);   
    saveConfig(CONFIG_WANIP+1,0);   
    saveConfig(CONFIG_WANIP+2,0);   
    saveConfig(CONFIG_WANIP+3,0);           
    
    saveConfig(CONFIG_VALID,0xAA);    
 
     }
