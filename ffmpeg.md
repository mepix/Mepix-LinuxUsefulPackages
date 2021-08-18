# FFMPEG

## Converting `.png` to `.mp4`

Ideally, the images are 0-padded.

```sh
# With Proper Zero Indexing
ffmpeg -r 1/5 -i img%03d.png -c:v libx264 -vf fps=25 -pix_fmt yuv420p out.mp4

# Using a Glob Pattern
ffmpeg -pattern_type glob -i '*.png' -vcodec libx264 -vf fps=30 -pix_fmt yuv420p out2.mp4

```

## Converting `.mov` to `.mp4`

```sh
ffmpeg -i {in-video}.mov -vcodec h264 -acodec aac {out-video}.mp4
```
