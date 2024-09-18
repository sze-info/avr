# [sze-info.github.io/avr](https://sze-info.github.io/avr)


🚘 Autonomous vehicles and robots software engineering || Autonóm járművek és robotok programozása 


## Acknowledgement

There are many courses who inspired this lecture, some parts were referenced or cited in this course with proper open source licencing.  

- We are grateful to our colleagues at the **MIT RACECAR** project for creating the F1/10 car model for Gazebo, and the F1/10 community for sharing insights on mapping, localization, and TEB-planner. [MIT license](https://choosealicense.com/licenses/mit/).
- Thank you for the F1/10 Crew at **Univerisity of Virginia**: Dr. Madhur Behl and Varundev Suresh Babu. [GPL-3.0 license](https://choosealicense.com/licenses/gpl-3.0/). [Link](https://github.com/linklab-uva/f1tenth_gtc_tutorial/blob/master/LICENSE).
- Thank you for **ETH Zürich** Programming for Robotics - ROS: Péter Fankhauser, Dominic Jud, Martin Wermelinger, Prof. Dr. Marco Hutter.
- Thank you for "Autonomous Driving Software Engineering" by the Institute of Automotive Technology (Prof. Dr.-Ing. Lienkamp, Phillip Karle), **TU München**. [GPL-3.0 license](https://github.com/TUMFTM/Lecture_ADSE/blob/master/LICENSE.md).
- Thank you for TH Ingolstadt, Autonomous Vehicle Engineering Faculty of Electrical Engineering and Computer Science.
- Thank you **Stanford University** - Introduction to Robotics by Prof. Oussama Khatib.
- Thank you **Stanford University** - CS231n: Convolutional Neural Networks for Visual Recognition. Andrej Karpathy. [MIT license](https://github.com/cs231n/cs231n.github.io/blob/master/LICENSE)
- Thank you **Autoware** Foundation for the software and the documentation. [Apache License 2.0](https://github.com/autowarefoundation/autoware-documentation/blob/main/LICENSE).
- Thank you **Óbuda University** Antal Bejczy Center for Intelligent Robotics - Tamás D. Nagy and Péter Galambos [CC BY-NC-SA 4.0](https://github.com/ABC-iRobotics/ros_course_materials_hu/blob/main/LICENSE.md)
- Thank you [Antonio Mauro Galiano](https://github.com/dottantgal/ros2_pid_library/) - [MIT license](https://github.com/dottantgal/ros2_pid_library/blob/main/LICENSE)
- Thank you Bokor József, [Bokor József & Gáspár Péter. Irányítástechnika](https://dtk.tankonyvtar.hu/bitstream/handle/123456789/3269/Bokor_Gaspar_Soumelidis_Iranyitastechnika_II.pdf)
- Thank you Korondi Péter: "Rendszertechnika: Integrált gépészeti és villamos rendszerek leírása irányításelméleti megközelítésben", egyetemi jegyzet, Budapesti Műszaki és Gazdaságtudományi Egyetem, Mechatronika, Optika és Gépészeti Informatika tanszék

# [sze-info.github.io/ajr](https://sze-info.github.io/ajr)


## Credits
A big thank you goes to the following people/groups:

- [MkDocs] for providing the software, to generate documentation.
- [squidfunk] for the [MkDocs Material Theme].
- [facelessuser] for the [PyMdown Extensions].

## Commands

``` r
$ git clone https://github.com/sze-info/avr/ && cd avr
$ pip install mkdocs mkdocs-material "mkdocs-material[imaging]"
$ mkdocs build
$ mkdocs serve
```
## Note
If you encounter the error `zsh: command not found: mkdocs` after installing `mkdocs-material`, it might be because the installation directory is not in your PATH. You can fix this by adding the following line to your shell configuration file (`.zshrc` for Zsh or `.bashrc` for Bash):

```sh
export PATH=$PATH:~/.local/bin
```
### After adding this line, reload your shell configuration:

```sh 
source ~/.zshrc  # For Zsh
source ~/.bashrc  # For Bash
```

## MkDocs License
MkDocs template is served under the MIT license.  
Read the [LICENSE] file for more info.
