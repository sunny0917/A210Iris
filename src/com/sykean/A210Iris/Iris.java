package com.sykean.A210Iris;

import android.util.Log;

public class Iris{

    static {
    	System.loadLibrary("A210Iris");
    }
    
    public native void setJNIEnv(Object obj,String func);
    public static native int openPort(String path,int baudrate);
    public static native int closePort();
    public static native int sendData(byte cmd,byte[] send_data,int nDatalen);
    //public static native int recvData(byte cmd, byte[] recv_data, int recv_len);

}
