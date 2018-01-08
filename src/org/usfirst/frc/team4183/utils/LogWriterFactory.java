package org.usfirst.frc.team4183.utils;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;

public class LogWriterFactory {

	private int serial = 0;
	private String fileNameRoot;
	
	public LogWriterFactory( String fileNameRoot) {
		this.fileNameRoot = fileNameRoot;
	}
	
	public class Writer {
		
		private long startMillis;
		private PrintWriter pw;
		
		private Writer( PrintWriter pw) {
			this.pw = pw;
			startMillis = System.currentTimeMillis();
		}
		
		public void writeLine( String line) {
			if( pw != null) {
				String timeStamp = String.format("%6d ", 
						System.currentTimeMillis() - startMillis);
				pw.println( timeStamp + line);
			}
		}
		
		public void close() {
			if( pw != null)
				pw.close();
		}		
	}
	
	public Writer create( boolean enable) {
				
		String fileName = String.format("%s_%d.txt", fileNameRoot, serial++);		
		File file = new File( getLogDir(), fileName);		
		PrintWriter pw = null;
		if( enable) {
			try {
				pw = new PrintWriter( file);
			} catch (FileNotFoundException e) {
				e.printStackTrace();
			}
		}
		return new Writer( pw);
	}
	
	public Writer create() {
		return create(true);
	}
	

	// Potential log directory locations --
	// /u, /v etc. will exist if USB plugged in;
	// otherwise use lvuser's home dir
	private final String[] roots 
	= new String[] {"/u", "/v", "/x", "/y", "/home/lvuser"};
	
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
}
