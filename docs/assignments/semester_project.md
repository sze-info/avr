---
title: Large Semester Project
permalink: /large_semester_project/
icon: material/school
---

# Large Semester Project

The preparation of the large semester project requires more time, but it allows for much more interesting tasks to be developed over many weeks. Based on the semester project, after completing the course, a thesis, diploma work, project work, or TDK paper can be prepared, and there is also an opportunity to complete the mandatory professional practice.

## Examples

Example of a large semester project created by the instructors:

- [github.com/horverno/simple_random_trees](https://github.com/horverno/simple_random_trees): The package is a simple random tree algorithm that can be used for path planning. This implementation focuses on visualization rather than a comprehensive random tree-based path planning system. The `/display_tree node` advertises a `/path_marker_topic`, which is of type `visualization_msgs/marker_array`. The functions implementing the tree data structure are placed in a separate header file. Implemented under `ROS 2 Humble`.

The following examples were not necessarily created as semester projects but would be acceptable as such:

- [github.com/jkk-research/wayp_plan_tools](https://github.com/jkk-research/wayp_plan_tools)
- [github.com/jkk-research/sim_wayp_plan_tools](https://github.com/jkk-research/sim_wayp_plan_tools)
- [github.com/jkk-research/pointcloud_to_grid](https://github.com/jkk-research/pointcloud_to_grid)
- [github.com/jkk-research/urban_road_filter](https://github.com/jkk-research/urban_road_filter)
- [github.com/dobaybalazs/curb_detection](https://github.com/dobaybalazs/curb_detection)
- [github.com/kkira07/Szakdolgozat](https://github.com/kkira07/Szakdolgozat)
- [github.com/szenergy/rviz_markers](https://github.com/szenergy/rviz_markers)
- [github.com/linklab-uva/f1tenth_gtc_tutorial](https://github.com/linklab-uva/f1tenth_gtc_tutorial)
- [github.com/Farraj007/Jkk-task](https://github.com/Farraj007/Jkk-task)
- [github.com/leander-dsouza/breakout_gazebo](https://github.com/leander-dsouza/breakout_gazebo)
- [github.com/fjp/ros-turtle-pong](https://github.com/fjp/ros-turtle-pong)

*Note*: In the course, we use the ROS 2 Humble version, but the semester project (with justification) can be accepted in other versions as well.

## Positive Aspects for the *Semester* Task:
- 👍 Well-documented in Hungarian and/or English, illustrated with images. Use of [Markdown](https://docs.github.com/en/get-started/writing-on-github/getting-started-with-writing-and-formatting-on-github/basic-writing-and-formatting-syntax).
- 👍 Basic information in the `README.md`, (optional) documentation in the `/wiki`.
- 👍 Issues.
- 👍 Branches.
- 👍 Gitignore.
- 👍 License.
- 👍 Repository topics, including the course code and SZE. Based on the topics, the repository can be listed here: [github.com/topics/sze](https://github.com/topics/sze).
- 👍 Extra points can be given if the current material is supplemented or corrected (of course, through a pull request).

## *Serious Mistakes* that can significantly lower the grade of the *Semester* project:
- 😡 Compressed files in the GitHub repository (e.g., `zip` and even worse if `rar`). An exception can be if the goal is to handle compressed files directly, but source code, images, etc., should never be uploaded this way.
- 😡 [Not original work](#meme), or the borrowed code is not referenced.
- 😡 In a team, only one student commits. (This obviously does not apply to single-person tasks).
- 😡 Few commits. The appropriate number of commits is important because it helps us judge how the workflow progressed, who worked on what and when.
- 😡 No `README.md`, missing short documentation or images.
- 😡 Documentation uploaded as pdf/docx instead of in the `/wiki`.
- 😡 File upload instead of commit.
- 😡 Source code screenshotted instead of using markdown syntax highlighting. (Since the code cannot be copied, searched, etc., as an image).

## Ideas for Topic Selection

- Inspiration can come from previous or current theses/diploma works: [horverno.github.io/temaajanlatok](https://horverno.github.io/temaajanlatok/)
- It is advisable to choose a topic that you would enjoy working on for weeks/months. If, for example, visualization, algorithm practice, 3D, or artificial intelligence is attractive, then it is advisable to choose a topic accordingly.
- Previous theses, semester projects are available and can be requested [here](https://docs.google.com/forms/d/e/1FAIpQLSdtMK--IQl4v5pHiATDP4MJwuU-M0Ycd2keMndQfuuhvlr1rA/viewform?usp=sf_link). It is important that these **must not** be shared further, they are available for educational purposes only.
- Numerous ROS 2 projects here: [github.com/fkromer/awesome-ros2](https://github.com/fkromer/awesome-ros2)

!!! tip
    It is highly recommended to obtain the [GitHub Student Developer Pack](https://education.github.com/pack), which includes [Copilot](https://github.com/features/copilot) among other things.

![](https://github.blog/wp-content/uploads/2019/08/FBLinkedIn_ALL-PARTNERS.png)

## Evaluation Criteria

The criteria were based on the evaluation criteria of a similar course at [Óbuda University](https://abc-irobotics.github.io/ros_course_materials_hu/#evkozi-jegy_1).
- Ratio of own work to used codebase (with proper references)
- Work producing evaluable results
- Quality of the presentation (ppt, videos, live demo, any additional tools used)
- Completeness of the solution
- Proper `ROS 2` communication / best practice application
- Structure of the program
- Quality of the implementation
- Documentation of the code
- Consultation
- Difficulty of the chosen task

## Recommended Method for Creating the Semester Project Repo: `Template`

We have created a so-called template repo in both C++ and Python, which makes it easier to create the first repository containing a package:

- [github.com/sze-info/ros2_cpp_template](https://github.com/sze-info/ros2_cpp_template)
- [github.com/sze-info/ros2_py_template](https://github.com/sze-info/ros2_py_template)

!!! info
    You can read about it [here](https://sze-info.github.io/arj/onallo/ros2git.html).

<img src="https://raw.githubusercontent.com/sze-info/ros2_cpp_template/main/img/use_this_template01.png" width="60%" />

### Meme

<center><img src="https://raw.githubusercontent.com/sze-info/arj/main/docs/feleves_beadando/meme01.jpg" width="60%" /></center>

Credit: [pycoders](https://www.instagram.com/pycoders/)

<center><img src="https://raw.githubusercontent.com/sze-info/arj/main/docs/feleves_beadando/meme02.jpg" width="60%" /></center>

Credit: [knowyourmeme](https://knowyourmeme.com/memes/but-its-honest-work)