package org.usfirst.frc.team4183.utils;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.lang.Thread;


public class ThreadLogger {

	// User must implement this interface
	public interface Client {
		public void writeLine( PrintWriter writer, long millis);
	}
	
	private static final long DEFAULT_INTERVAL = 10;     // Logging interval msec
	private static final double DEFAULT_DURATION = 10.0; // Seconds

	private final Client client;
	private final long interval;    // Millis
	private final double duration;  // Seconds			
	private final File file;

	private PrintWriter writer;
	private WriterThread writerThread = null;
	
	// Create Logger
	// _interval: milliseconds
	// _duration: seconds
	public ThreadLogger( Client _client, String fileName, long _interval, double _duration) {		
		client = _client;
		interval = _interval;
		duration = _duration;

		file = getLogFile( fileName);
		System.out.format("Logger created using %s\n", file);	
	}

	// Create logger w/ default interval & duration
	public ThreadLogger( Client _client, String _fileName) {
		this( _client, _fileName, DEFAULT_INTERVAL, DEFAULT_DURATION);
	}

	public void start() {

		if( writerThread != null && writerThread.isAlive()) {
			System.err.println( "Logger already running, start() ignored");
			return;
		}

		try {
			writer = new PrintWriter( file);
			
			writerThread = new WriterThread(writer, interval, duration);
			writerThread.setPriority(Thread.NORM_PRIORITY+2);
			writerThread.start();
		}
		catch (FileNotFoundException e) {
			if( writer != null) writer.close();			
			e.printStackTrace();
		}
	}

	public void stop() {
		writerThread.quit();
		writerThread = null;
	}


	private File getLogFile( String fileName) {		
		File file = new File( getLogDir(), fileName);
		return file;
	}

	// Potential log directory locations --
	// /u, /v etc. will exist if USB plugged in;
	// otherwise use lvuser's home dir
	private final String[] roots 
	= new String[] {"/u", "/v", "/x", "/y", "/home/lvuser"};

	// Creates & returns appropriate log directory.
	// Tries the potential root locations given in the "roots" String array.
	// Creates "logs" subdir in the first root that exists.
	private File getLogDir() {
		File dir;

		// Try each directory location
		for( String r : roots) {
			File root = new File(r);
			if( root.isDirectory()) {
				dir = new File( root, "logs");
				dir.mkdirs();
				if(dir.isDirectory())
					return dir;
			}
		}

		return null;
	}
	
	private class WriterThread extends Thread {

		private PrintWriter writer;
		private long interval;
		private double duration;
		private long startMillis;

		WriterThread( PrintWriter _writer, long _interval, double _duration) {			
			writer = _writer;
			interval = _interval;
			duration = _duration;
			startMillis = System.currentTimeMillis();
		}

		public void quit() {
			interrupt();
		}
		
		public void run() {
			
			while(!isInterrupted()) {
				long millis = System.currentTimeMillis() - startMillis;
				if( millis > 1000.0 * duration) 
					break;
				
				client.writeLine( writer, millis);
				
				try {
					sleep(interval);
				} catch (InterruptedException e) {
					interrupt();
				}
			}
			
			writer.close();
		}
	}

}
