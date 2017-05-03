package com.test;

import com.sykean.A210Iris.Iris;
import com.sykean.A210Iris.R;

import android.os.Bundle;
import android.app.Activity;
import android.util.Log;
import android.view.Menu;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;

public class IrisActivity extends Activity implements OnClickListener {

	private Button bt_start_signal;
	private Button bt_get_status;
	private Button bt_start_model;
	private Button bt_get_model;
	private Button bt_stop_model;

	static Iris irsi = null;
	
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        //nativeInit();
        initView();
        
        irsi = new Iris();
    	irsi.setJNIEnv(this,"recvCallback");
        
        irsi.openPort("/dev/ttyS7",115200);
        
    }

    private void initView() {
    	bt_start_signal = (Button) findViewById(R.id.bt_start_signal);
    	bt_get_status = (Button) findViewById(R.id.bt_get_status);
    	bt_start_model = (Button) findViewById(R.id.bt_start_model);
    	bt_get_model = (Button) findViewById(R.id.bt_get_model);
    	bt_stop_model = (Button) findViewById(R.id.bt_stop_model);
    	bt_start_signal.setOnClickListener(this);
    	bt_get_status.setOnClickListener(this);
    	bt_start_model.setOnClickListener(this);
    	bt_get_model.setOnClickListener(this);
    	bt_stop_model.setOnClickListener(this);
    }
    
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.main, menu);
        return true;
    }
    
    private static void recvCallback(byte cmd,byte[] info,int len) {
    	Log.e("A210Iris","recvCallback cmd="+cmd+" len="+len);
    	for(int i = 0;i < len;i++)
    		Log.e("A210Iris","info["+i+"]="+info[i]);
    }

	@Override
	public void onClick(View arg0) {
		// TODO Auto-generated method stub
		switch (arg0.getId()) {
		case R.id.bt_start_signal:
	        byte[] none = new byte[1024];
	        irsi.sendData((byte)0x41, none, 0);
			break;
		case R.id.bt_get_status:
	        byte[] none1 = new byte[1024];
	        irsi.sendData((byte)0x47, none1, 0);
			break;
		case R.id.bt_start_model:
	        byte[] none11 = new byte[1024];
	        irsi.sendData((byte)0x4D, none11, 0);
			break;
		case R.id.bt_get_model:
	        byte[] none111 = new byte[1024];
	        irsi.sendData((byte)0x4E, none111, 0);
			break;
		case R.id.bt_stop_model:
	        byte[] none1111 = new byte[1024];
	        irsi.sendData((byte)0x51, none1111, 0);
			break;

		default:
			break;
		}
	}
}
