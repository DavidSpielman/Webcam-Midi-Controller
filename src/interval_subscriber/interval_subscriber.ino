#include <ros.h>
#include <std_msgs/Int16.h>
#include <Control_Surface.h>

ros::NodeHandle nh; //initialize ros node
USBMIDI_Interface midi; //Instantiate MIDI over USB interface

const MIDIAddress note = 0;
const uint8_t velocity = 0x7F; // The velocity of the note events
bool note_on = false;

void interval_msg(const std_msgs::Int16& msg){
  if(msg.data == 1) {
    const MIDIAddress note {MIDI_Notes::C(4), CHANNEL_1};
    note_on = true;
  }
  else if (msg.data == 2) {
    const MIDIAddress note {MIDI_Notes::D(4), CHANNEL_1};
    note_on = true;
  }
  else if (msg.data == 3) {
    const MIDIAddress note {MIDI_Notes::E(4), CHANNEL_1};
    note_on = true;
  }
  else if (msg.data == 4) {
    const MIDIAddress note {MIDI_Notes::F_(4), CHANNEL_1};
    note_on = true;
  }
  else if (msg.data == 5) {
    const MIDIAddress note {MIDI_Notes::G(4), CHANNEL_1};
    note_on = true;
  }
  else if (msg.data == 6) {
    const MIDIAddress note {MIDI_Notes::A(4), CHANNEL_1};
    note_on = true;
  }
  else if (msg.data == 7) {
    const MIDIAddress note {MIDI_Notes::B(4), CHANNEL_1};
    note_on = true;
  }
  else if (msg.data == 0) {
    note_on = false;
  }
}
  
ros::Subscriber<std_msgs::Int16> sub("interval", &interval_msg);

void setup()
{
  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(sub);
  midi.begin(); // initializes the MIDI interface
}

void loop()
{
  nh.spinOnce();
  delay(500);
  //interval_msg();  Need a way to constantly check which note has been selected within the loop
  if (note_on == true)
    midi.sendNoteOn(note, velocity);
  else 
    midi.sendNoteOff(note,velocity);
  midi.update(); //read and handle or discard MIDI input

}
