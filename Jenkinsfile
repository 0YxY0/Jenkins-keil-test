pipeline {
	agent any
	stages {
		stage('Build') {
			steps {
				bat '''@echo off
					set UV=E:\\Keil5\\UV4\\UV4.exe
					set UV_PRO_PATH="C:\\Users\\yxyxi\\Desktop\\test\\ch395_static_ip_test\\project.uvprojx"
					echo Init building ...
					echo .>build_log.txt
					%UV% -j0 -r %UV_PRO_PATH% -o %cd%\\build_log.txt
					type build_log.txt
					echo Done.
					pause'''
			}
		}
	}
}