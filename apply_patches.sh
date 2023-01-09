#!/bin/bash
found=false;
for i in `ls openwrt_patches/pending-5.10`
do
	if [ "$found" = false ]; then
		if [[ "$i" =~ ^483-mtd-spinand-add-support-for-xtx-xt26g0xa.* ]]; then
			found=true
		fi
		continue
	fi
	echo $i
	git am openwrt_patches/pending-5.10/$i;
	retval=$?;
	if [ $retval -eq 128 ]; then
		git apply openwrt_patches/pending-5.10/$i || break
		git add . || break
		git commit -m "openwrt_patches/pending-5.10/$i" || break
		continue
	elif [ $retval -eq 0 ]; then
		continue
	else
		break
	fi
	echo "SF"
	break
done
