package com.sykean.A210Iris;

import android.util.Log;

public class Iris{

    static {
    	System.loadLibrary("A210Iris");
    }
    
    /**
     * 设置JNI回调函数
     * @param obj	上下文
     * @param func	回调函数名(recvCallback)
     */
    public native void setJNIEnv(Object obj,String func);
    
    /**
     * 初始化资源(打开串口，启动线程)
     * @param path	串口名
     * @param baudrate	波特率(115200)
     * @return	0：初始化成功	-1:初始化串口失败	-2:创建线程失败
     */
    public static native int Init(String path,int baudrate);
    
    /**
     * 释放资源(关闭串口，停止线程)
     * @return	0:成功	-1:释放串口失败
     */
    public static native int Destory();
    
    /**
     * 启动信号
     * @return	0:写串口成功	-1:写串口失败
     */
    public static native int Start();

    /**
     * 
     * 启动识别
     * @param userId	用户ID(userid全为0则表示1：N识别（当通过测距触发自动识别时，设备默认为1：N识别，并返回识别结果），userid为全FF表示循环识别，否则只对 userid 进行1:1识别。)
     * @param nLen		24位用户ID
     * @return			0:写串口成功	-1:写串口失败
     */
    public static native int Recog(byte[] userId,int nLen);

    /**
     * 启动注册
     * @param userId	用户ID
     * @param nLen		24位用户ID
     * @return			0:写串口成功	-1:写串口失败
     */
    public static native int Register(byte[] userId,int nLen);

    /**
     * 删除模板
     * @param userId	用户ID(userid为24个0删除全部本地存储的虹膜信息，否则删除对应userid的虹膜信息)
     * @param nLen		24位用户ID
     * @return			0:写串口成功	-1:写串口失败
     */
    public static native int Delete(byte[] userId,int nLen);
    
    /**
     * 获取本地用户
     * @param userId	用户ID(传入24位userid返回当前用户，全0返回全部用户)
     * @param nLen		24位用户ID
     * @return			0:写串口成功	-1:写串口失败
     */
    public static native int Read(byte[] userId,int nLen);
    
    /**
     * 取消当前操作
     * @return	0:写串口成功	-1:写串口失败
     */
    public static native int Cancle();
    
    /**
     * 获取当前设备状态
     * @return	0:写串口成功	-1:写串口失败
     */
    public static native int Sync();
    
    /**
     * 获取模板
     * @param userId	用户ID
     * @param nLen		24位用户ID
     * @return	0:写串口成功	-1:写串口失败
     */
    public static native int Get(byte[] userId,int nLen);
    
    /**
     * 下发模板
     * @param data	userId+1024bytes虹膜模板数据
     * @param nLen	数据长度
     * @return	0:写串口成功	-1:写串口失败
     */
    public static native int Down(byte[] app,int nLen);
    
    /**
     * 升级应用
     * @param app	appdata+crc
     * @param nLen	应用长度+4byteCRC
     * @return	0:写串口成功	-1:写串口失败
     */
    public static native int App(byte[] app,int nLen);
    
    /**
     * 切换识别模式
     * @param mode	0x01:切换到卡识别模式	0x02:切换到本地识别模式
     * @param nLen	0x01
     * @return	0:写串口成功	-1:写串口失败
     */
    public static native int Mode(byte mode,int nLen);
    
    /**
     * 读卡号
     * @return	0:写串口成功	-1:写串口失败
     */
    public static native int ReadCard();
    
    /**
     * 捕捉图像
     * @return	0:写串口成功	-1:写串口失败
     */
    public static native int Image();
    
    /**
     * 下发配置信息
     * @param config	配置信息(前3个byte表示注册质量，分别为：张合度（默认50），清晰度（默认60），斜视度（默认50），上述3个参数取值范围在40----90之间，若下发为 FF FF FF 则表示使用默认值，不重新设置；
	 *							后3个byte表示识别质量，分别为张合度（默认50），清晰度（默认40），斜视度（默认50），上述3个参数取值范围在40----90之间，若下发为FF FF FF 则表示使用默认值，不重新设置；)
     * @param nLen	0x06
     * @return	0:写串口成功	-1:写串口失败
     */
    public static native int Config(byte[] config,int nLen);
    
    /**
     * 开始产生模板
     * @return	0:写串口成功	-1:写串口失败
     */
    public static native int BuildStart();
    
    /**
     * 获取实时模板
     * @return	0:写串口成功	-1:写串口失败
     */
    public static native int Build();
    
    /**
     * 结束产生实时模板
     * @return	0:写串口成功	-1:写串口失败
     */
    public static native int BuildEnd();
}
