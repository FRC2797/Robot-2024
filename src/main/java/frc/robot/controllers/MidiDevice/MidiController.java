package frc.robot.controllers.MidiDevice;

import java.util.ArrayList;
import java.util.List;

import javax.sound.midi.MidiDevice;
import javax.sound.midi.MidiMessage;
import javax.sound.midi.MidiSystem;
import javax.sound.midi.MidiUnavailableException;
import javax.sound.midi.Receiver;
import javax.sound.midi.Transmitter;

public class MidiController
{
    private static final String MIDI_DEVICE_INFO = "DDJ-SB3";

    public MidiController()
    {
        System.out.println("All Midi Devices:");

        List<MidiDevice> devices = new ArrayList<MidiDevice>();
        MidiDevice.Info[] infos = MidiSystem.getMidiDeviceInfo();
        for (int i = 0; i < infos.length; i++) {
            System.out.println("- " + infos[i].toString());
            if (infos[i].toString().equals(MIDI_DEVICE_INFO))
            {
                try
                {
                    devices.add(MidiSystem.getMidiDevice(infos[i]));
                } catch (MidiUnavailableException e) {}
            }
        }
 
        if (devices.size() == 0)
        {
            System.out.println("Could not open " + MIDI_DEVICE_INFO);
            return;
        }

        MidiDevice device = devices.get(1);

        try
        {
            device.open();
        } catch (MidiUnavailableException e) { return; }
        System.out.println(MIDI_DEVICE_INFO + " opened!");
        
        System.out.println("Device Info: " + device.getDeviceInfo());
        System.out.println("Transmitters: " + device.getTransmitters());
        System.out.println("Receivers: " + device.getReceivers());

        Transmitter transmitter = null;
        try
        {
            transmitter = device.getTransmitter();
        } catch (MidiUnavailableException e) { return; }

        transmitter.setReceiver(new MyReceiver());

        System.out.println("Device Info: " + device.getDeviceInfo());
        System.out.println("Transmitters: " + device.getTransmitters());
        System.out.println("Receivers: " + device.getReceivers());
    }

    static public class MyReceiver implements Receiver {
        public MyReceiver() {
            left_value = 0;
            right_value = 0;
        }
        public void send(MidiMessage msg, long timeStamp) {
            byte[] msg_bytes = msg.getMessage();
            if (msg_bytes[0] == -80 && msg_bytes[1] == 33) // Left Turntable
            {
                if (msg_bytes[2] == 65)
                {
                    left_value++;
                }
                else
                {
                    left_value--;
                }
                System.out.println("Left: " + left_value);
            }
            else if (msg_bytes[0] == -79 && msg_bytes[1] == 33) // Right Turntable
            {
                if (msg_bytes[2] == 65)
                {
                    right_value++;
                }
                else
                {
                    right_value--;
                }
                System.out.println("Right: " + right_value);
            }
            else
            {
                if (msg_bytes[2] == 127)
                {
                    System.out.println("Control (" + msg_bytes[0] + "," + msg_bytes[1] + ") ON");
                }
                else if (msg_bytes[2] == 0)
                {
                    System.out.println("Control (" + msg_bytes[0] + "," + msg_bytes[1] + ") OFF");
                }
                else
                {
                    System.out.println("Control (" + msg_bytes[0] + "," + msg_bytes[1] + ") Value: " + msg_bytes[2]);
                }
            }

            //Raw bytes
            // for (int i = 0; i < msg_bytes.length; i++)
            // {
            //     System.out.println(i + " " + msg_bytes[i]);
            // }
        }
        public void close() {}
        private int left_value;
        private int right_value;
    }
};