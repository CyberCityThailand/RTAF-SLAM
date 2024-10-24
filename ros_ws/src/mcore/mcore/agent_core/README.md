# agent_core

The agent_core repository is intended to be a starting point to get your carrier-board logic connected to a Cube and operating in the real world.  This repository contains multiple files that you may want to edit but the my_agent.py file is the file you will run on your development system or carrier-board.

To leverage the model-simulate-test philosophy, the codebase here has the ability to operate with either an Ardupilot simulator or with the Cube directly.  If the development system or carrier-board has a Cube connected to it, the code will automatically recognize the Cube and configure itself to talk to the Cube.  If a Cube is not connected, it will look for an active Ardupilot simulator at the TCP/UDP address and port you dictate in the SIMULATOR_ADDRESS parameter in your agent_configuration.yaml file.

### This repository is a Git Template
This repository is designed as a Template Repository.  This means that it is not intended to be chaned but to act as a starting point for your own repository where you will make changes to fit your vehicle modality and purpose.  The DFEC instructors will evolve this repository over time to give it more functions and capability.  You will be able to then update your code when these changes occur but you will not include your changes back to this repository but to the repository you created from this template.

You have two approaches for using the agent_core codebase:
  - Clone the template into your own repository
  - Download the files from the repository

### Cloning This Template
  1. From within your own GitHub account, follow the instructions <a href="https://www.markdownguide.org" target="_blank">here</a>.  This will create a new repository in your account that will contain all the files and structure of this agent_core repository.
  2. From your development computer (if developing code outside of the carrier-board), clone your new repository using your local the git install on the development computer.
  3. From your carrier-board, clone your new repository using the git install on your carrier-board.
  4. As you make changes to your code on your development computer, commit to your local repository and then push to your GitHub repository for distrobution.
  5. On your carrier-board pull the changes from your Git repository into your carrier-board for live execution.
  
### Download Files
  1. A simplier but possibly riskier approach as far as version control goes is to download the repository to your development computer/carrier board and make edits directly.
  2. You will be responsible for ensuring consistency across your development and operational hardware as well as between team members.
  
## Setting up dependencies for agent_core

  1. Import the files as described above to your desired operating directory.
  2. If you are operating within a python environment, make sure it's active in the desired operating folder.
  3. To install all the required libraries, from within the environment and the directory you cloned/downloaded the agent_core codebase, type:
  
      `pip install -r requirements.txt`
   
  4. To run the code, from within the root directory of the cloned/downloaded files type:
  
      `python my_agent.py`
    
