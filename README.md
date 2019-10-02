# Master Project 2019

### Introduction

This is my master project on automated geographical alignment procedure through dense point cloud simplification. Current state of the art algorithms do perform well when aligning relatively simple point clouds. However when the number of points and the complexity of clouds increases, existing algorithms tend to get stuck in local minimas and never find the optimal solution. This is due to the fact that the number of points and topological of the cloud becomes too complex and the number of keypoints involved to optimize the cost function is to high for it to make sense, hence the erratical results.

In order to solve this problem, the idea explored in this project is to perform the registration on a simplification of the original source and target clouds. The type of clouds at my disposal during this project was ones of the city of Geneva, recorded by aerial view using a LIDAR technology. It is mainly formed by planar surfaces representing the roofs and walls of the city's buildings, as well as the roads. For this reason, the simplification choosed was to segment the point clouds of both the source and target clouds in planes. These formed two sets of segmented planes, each defined by their plane's center coordinates, their normal vector and the points belonging to each of them. The registration algorithm is then performed using these two sets of planes, that are extremely simplified rather than the original clouds.

### Research summary



### Installation and Usage

- Dependencies: The code has been developped on Linux Mint. The external libraries are as follows:
	- pcl 1.9
	- omp
	- boost


### License

Automated geographical alignment procedure through dense point clouds simplification - Loris Aiulfi
Copyright (c) 2019 EPFL
This program is licensed under the terms of the [license]. // AGPL, GPL, LGPL or MIT_  


# Student Project Template

Please name your repository as follows:
- use lower case
- use hyphens to separate tokens
- if related to a larger project, start with the name of this project, followed by the name of your project (e.g. 'impresso-image-classification')
- in case of doubt ask your supervisors

Please structure your repository as follows:

- a folder **code** where you put... your code and your resources
- a folder **report** where you put... your report
- a **README**, with the following information:

### Introduction
_brief introduction of the project_

### Research summary
_brief summary of your approaches/implementations_
_illustration of results_

### Installation and Usage
- _dependencies: platform, libraries_
- _compilation (if necessary)_
- _usage: how to run your code_
