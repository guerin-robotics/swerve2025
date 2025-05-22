

### Mac Setup (WIP)

Download WPILib VS Code https://.....

Install: Extensions -> Extension Pack for Java

### Logging Sensors
https://docs.advantagekit.org/getting-started/installation/existing-projects/

Cmd+Sht+P -> Mange Vendor Libraries -> Install Online
https://github.com/Mechanical-Advantage/AdvantageKit/releases/latest/download/AdvantageKit.json

### Building
Verify this file is udpated with latest timestamp:
 /Users/parkerpulfer/swerve2025/build/libs/swerve2025.jar
Deploying Code




### ssh to robot

make sure on the 10021 wifi network

ssh admin@@10.100.21.2
<no password>


### kill java on robot

ps -af | grep swerve
kill <pid>

### TODO
* Find and use java style guide (lint)
* consistant member variable naming convention (don't use '''m_''')
* consistant initialization (member var initializated in constructor)
* consistant class member order: static var, constructor first method
* update logger: add per topic/file logging level

