# FFMPEG

## Resize Video

```sh
ffmpeg -i input.mov -vf scale=640:360 output.mov
```

## Converting `.png` to `.mp4`

Ideally, the images are 0-padded.

```sh
# With Proper Zero Indexing
ffmpeg -r 1/5 -i img%03d.png -c:v libx264 -vf fps=25 -pix_fmt yuv420p out.mp4

# Using a Glob Pattern
ffmpeg -pattern_type glob -i '*.png' -vcodec libx264 -vf fps=30 -pix_fmt yuv420p out2.mp4

```

## Converting `.mp4` to `.png`

**Note:** the `fps` flag sets how many frames to be created per second of video. In the below examples, this will create one image per second.

```sh
ffmpeg -i input.mp4 -vf fps=1 out%d.png
```

## Converting `.mov` to `.mp4`

```sh
ffmpeg -i {in-video}.mov -vcodec h264 -acodec aac {out-video}.mp4
```

## Converting `.mov` to `.gif`

```sh
ffmpeg -ss 30 -t 3 -i input.mp4 -vf "fps=10,scale=320:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" -loop 0 output.gif
```
From [Stackoverflow](https://superuser.com/questions/556029/how-do-i-convert-a-video-to-gif-using-ffmpeg-with-reasonable-quality):

- `-ss` skip the first 30 seconds of the input
- `-t` create a 3 second output
- `fps` filter sets the frame rate. A rate of 10 frames per second is used in the example.
- `scale` filter will resize the output to 320 pixels wide and automatically determine the height while preserving the aspect ratio. The lanczos scaling algorithm is used in this example.
- `palettegen` and `paletteuse` filters will generate and use a custom palette generated from your input. These filters have many options, so refer to the links for a list of all available options and values. Also see the Advanced options section below.
- `split` filter will allow everything to be done in one command and avoids having to create a temporary PNG file of the palette.
- `-loop` output option but the values are confusing. A value of 0 is infinite looping, -1 is no looping, and 1 will loop once meaning it will play twice. So a value of 10 will cause the GIF to play 11 times.

## Compressing Video

A reasonable range for `H.26X` may be 24 to 30. Note: lower CRF values correspond to higher bitrates, and hence produce higher quality videos.

```sh
ffmpeg -i input.mov -vcodec libx264 -crf 30 -pix_fmt yuv420p output.mp4
```

From [Stackoverflow](https://unix.stackexchange.com/questions/28803/how-can-i-reduce-a-videos-size-with-ffmpeg)

## Manipulating `.gif`

```sh
# Compress GIF
gifsicle -O3 --colors 256 --lossy=30 -o output.gif input.gif

# Crop GIF
gifsicle --crop 109,0+281x281 --resize 150x150 original.gif \
  > resized-cropped-centered.gif

```
