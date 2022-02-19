# vox_nav
![foxy](https://github.com/jediofgever/vox_nav/workflows/foxy/badge.svg)  

Documentation is here ; https://vox-nav.readthedocs.io/en/latest/
 
### Videos 

You can download videos and see capablities of project. See all available videos under docs/assets.

* ![MPC following a Trajectory](docs/assets/mpc_3.mp4)

* ![Full Navigation using Behaviour Trees](docs/assets/navigation_in_action.mp4)

* ![Full Navigation DUBINS space](docs/assets/navigation_dubins_space.mp4)

* ![Full Navigation SE3 space](docs/assets/navigation_se3_planner.mp4)

* ![Full Navigation REEDSPEEP space](docs/assets/navigation_se2_control_planner.mp4)

* ![Thorvald Navigation with vox_nav](docs/assets/real_robot_demos.mp4)

[![Thorvald Navigation with vox_nav](https://img.youtube.com/vi/16H4n_H7RzI/0.jpg)](https://www.youtube.com/watch?v=16H4n_H7RzI)

<!DOCTYPE html>
<html>
<head>
	<meta charset="utf-8">
	<title>Local HTML5 Video Player</title>
</head>
<body>

<style>
	* { box-sizing: border-box; }
	body { margin: 0; font-family: sans-serif; }

	video {
		z-index: 1;
		display: block;
		margin: 0 auto;
		background-color: #ccc;

		/* maximize video player size within viewport while maintaining 16:9 format */
		width: 100vw;
		height: 56.25vw;
		max-width: 177.78vh;
		max-height: 100vh;
	}

	@keyframes fadeOut {
		0%   { opacity: 1; }
		95%  { opacity: 1; }
		100% { opacity: 0; }
	}
	.fadeout { animation: fadeOut 1s linear; }

	input, ul, a {
		position: absolute;
		z-index: 2;
		opacity: 0;
		transition: opacity 0.15s ease-out;
	}

	input:hover, ul:hover, a:hover { opacity: 1; }

	input[type=file] { display: none; }

	a {
		display: block;
		width: 100%;
		background-color: rgba(0, 0, 0, 0.7);
		text-align: center;
		text-decoration: none;
		text-transform: uppercase;
		color: white;
		left: 0;
		top: 0;
		padding: 2em;
		font-size: 1.5em;
	}

	input[type=text] {
		bottom: 1.5em;
		left: 1em;
		font-size: 1.5em;
		width: 4em;
		text-align: center;
		border: 1em solid rgba(0, 0, 0, 0.7);
	}

	ul {
		margin: 0;
		padding: 0 0 0 1em;
		top: 0;
		right: 0;
		color: white;
		background-color: rgba(0, 0, 0, 0.7);
		list-style: none;
	}

	li {
		margin: 0.5em;
		cursor: pointer;
	}
	li.played {
		text-decoration: line-through;
		list-style: square;
	}
</style>

<video controls autoplay></video>
<input type="file" multiple>
<a class='fadeout' href="#">Open Files</a>
<input class='fadeout' type="text" placeholder="playback speed" value="2">
<ul></ul>

<script>
	// get DOM elements
	const video = document.querySelector('video');
	const filesInput = document.querySelector('input[type=file]');
	const speedInput = document.querySelector('input[type=text]');
	const filesButton = document.querySelector('a');
	const playlist = document.querySelector('ul');

	// redirect filesButton click to hidden filesInput
	filesButton.addEventListener('click', e => {
		filesInput.click();
		e.preventDefault();
		return false;
	});

	filesInput.addEventListener('change', function (e) {
		// delete all current list items in playlist
		playlist.innerHTML = '';

		// go through all selected files
		for (const file of Array.from(this.files)) {

			// create list item and object url for the video file
			const listItem = document.createElement('li');
			listItem.objUrl = URL.createObjectURL(file);
			listItem.textContent = file.name;

			// give list item a click event listener for the corresponding video
			listItem.addEventListener('click', function (e) {
				this.classList.add('played');
				video.src = this.objUrl;
				video.playbackRate = Number(speedInput.value);
			});

			// append li to the list
			playlist.appendChild(listItem);
		};

		// show the playlist for a moment
		playlist.classList.add('fadeout');
	}, false /* don't capture */);

	// remove playlist fadeout after the animation ends, so it can be retriggered
	playlist.addEventListener('animationend', e => {
		playlist.classList.remove('fadeout');
	});

	// handle changes to speed input
	speedInput.addEventListener('change', e => {
		video.playbackRate = Number(speedInput.value);
		// write actual playback rate value back to input
		speedInput.value = Number(video.playbackRate);
	});

	// add keyboard shortcuts for pause (space) and 5 sec jump (left/right arrow)
	document.addEventListener('keydown', e => {
		// console.log(e.keyCode);
		switch (e.keyCode) {
			case 32: // space
				video.paused ? video.play() : video.pause();
				break;
			case 37: // left arrow
				video.currentTime += -5;
				break;
			case 39: // right arrow
				video.currentTime += 5;
				break;
		}
	});
</script>

</body>
</html>

### Related Publications

If using vox_nav for scientific publications, please consider citing the following paper.

```bash
@article{DBLP:journals/corr/abs-2103-13666,
  author    = {Fetullah Atas and
               Lars Grimstad and
               Grzegorz Cielniak},
  title     = {Evaluation of Sampling-Based Optimizing Planners for Outdoor Robot
               Navigation},
  journal   = {CoRR},
  volume    = {abs/2103.13666},
  year      = {2021},
  url       = {https://arxiv.org/abs/2103.13666},
  archivePrefix = {arXiv},
  eprint    = {2103.13666},
  timestamp = {Wed, 07 Apr 2021 15:31:46 +0200},
  biburl    = {https://dblp.org/rec/journals/corr/abs-2103-13666.bib},
  bibsource = {dblp computer science bibliography, https://dblp.org}
}
```

### Credits

* A lot of architectural aspects of this project has been inspired by the [Navigation2.](https://github.com/ros-planning/navigation2).
We greatly appreciate the efforts of [Navigation2.](https://github.com/ros-planning/navigation2) community for providing such high quality software design to Robotics community.

* This systems relies on libraries e.g. [OMPL](https://github.com/ompl/ompl), [Casadi](https://github.com/casadi/casadi), [Octomap](https://github.com/OctoMap/octomap)
  and many more, they cant all be listed here, we appriciate and recognize the efforts of people involved in development of these libraries.
