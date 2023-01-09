#!/bin/bash
found=false;
for i in `ls openwrt_patches/hack-5.10`
do
	# if [ "$found" = false ]; then
	# 	if [[ "$i" =~ ^483-mtd-spi-nor-add-gd25q512.patch.* ]]; then
	# 		found=true
	# 	fi
	# 	continue
	# fi
	echo $i
	git am openwrt_patches/hack-5.10/$i;
	retval=$?;
	if [ $retval -eq 128 ]; then
		git apply openwrt_patches/hack-5.10/$i || break
		git add . || break
		git commit -m "openwrt_patches/hack-5.10/$i" || break
		continue
	elif [ $retval -eq 0 ]; then
		continue
	else
		break
	fi
done
