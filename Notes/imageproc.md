# ImageMagick

## Image Scaling

```sh
convert myfigure.png -resize 200x100 myfigure.jpg
convert -resize 50% myfigure.png myfigure.jpg
```

## Crop

WIDTH x HEIGHT + OFFSET_X + OFFSET_Y 

```sh
convert original.jpg -crop 640x620+0+0 cropped.jpg
```

## Image Montage

To create a [multi-panel image](https://superuser.com/questions/290656/combine-multiple-images-using-imagemagick):

```sh
# Vertical Montage
montage -mode concatenate -tile 1x in-*.jpg out.jpg

# Horizontal Montage
montage -mode concatenate -tile x1 cropped*.png merge.png
```
## Compressino

```sh
convert -strip -interlace Plane -gaussian-blur 0.05 -quality 85% source.jpg result.jpg
```

# gThumb

Useful tool for basic image manipulation with a GUI

```sh
sudo apt install gthumb
```
