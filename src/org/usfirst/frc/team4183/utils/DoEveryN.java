package org.usfirst.frc.team4183.utils;

public class DoEveryN {
	
	private int N;
	Runnable runnable;
	private int cnt = 0;
	
	public DoEveryN( int N, Runnable runnable) {
		this.N = N;
		this.runnable = runnable;
	}
	
	public void update() {
		if( ++cnt == N) {
			cnt = 0;
			runnable.run();
		}
	}
}
