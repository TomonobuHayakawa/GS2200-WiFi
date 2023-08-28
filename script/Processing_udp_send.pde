import hypermedia.net.*;
import controlP5.*;

UDP udp;
ControlP5 cp5;

final String IP = "192.168.2.102";
final int PORT = 10002;

String msg = "test_messege";

void setup() {
  size(200, 200);

  cp5 = new ControlP5(this);
  udp = new UDP( this, 10001 );

  ControlFont cf = new ControlFont(createFont("メイリオ",20));

  cp5.addButton("UDP_Msg")
    .setFont(cf)
    .setLabel("送信")
    .setPosition(50,50)
    .setSize(100,100);
    
    udp.listen( true );
}

void draw() {
  background(200);
}

void UDP_Msg(){
  udp.send(msg,IP,PORT);
}

void receive( byte[] data, String ip, int port ) {
  String message = new String( data );
  println( "receive: \""+message+"\" from "+ip+" on port "+port );
}

