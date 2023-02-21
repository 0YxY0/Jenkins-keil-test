pipeline {
	agent any
	stages {
		stage('Build') {
			steps {
				powershell 'start keil_build.bat'
			}
		}
	}
}