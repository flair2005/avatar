#!/bin/bash
# by Arnaud Ramey (arnaud.a.ramey@gmail.com)
if [ $# -lt 2 ] || [ $# -gt 4 ]
then
  echo "Export all layers chosen by the user of a SVG file into PNG."
  echo
  echo "    Synopsis: $0 SVGFILE       LAYERSGROUPS             [OUTFILEPREFIX] [INKSCAPEARG]"
  echo "    Example:  $0 drawing.svg   bg+01,bg+02,bg+03,bg+04"
  echo "SVGFILE is the SVG to export. It is not modified."
  echo "LAYERSGROUPS is a comma-separated list of groups of layers."
  echo "  Each group of layers describes layers that must be exported together."
  echo "  Inside a group, layers are separated with a '+'."
  echo "INKSCAPEARG is not compulsoory but can be used eg with \"-d 300\" "
  echo "  to send the png resolution or any other inkscape switch through"
  exit
fi

TMPSVGORIG=$1
LAYERSGROUPS=$2
if [ $# -lt 3 ]; then
  OUTFILEPREFIX="$TMPSVGORIG_"
else
  OUTFILEPREFIX=$3
fi
INKSCAPEARG=$4
TMPSVG=`mktemp /tmp/fooXXXX.svg`
echo " *** $0: infile:'$TMPSVGORIG', outfiles:'$OUTFILEPREFIX*.png'"

set_layer_visible() { # 1: file, 2: layer name
  # https://stackoverflow.com/questions/7837879/xmlstarlet-update-an-attribute
  #~ echo "set_layer_visible($2@$1)"
  xmlstarlet edit --inplace --update "//*[@inkscape:label=\"$2\"]/@style" --value "display:inline" $1
} # end set_layer_visible()


# make all layers invisible
cp $TMPSVGORIG $TMPSVG
sed -i 's,style="display:inline",style="display:none",g' $TMPSVG

# for all userlayers
FRAMEIDX=1
IFS=','; for CURRENTLAYERSGROUP in $LAYERSGROUPS; do
  CURRENTSVG=${TMPSVG}_$FRAMEIDX.svg
  CURRENTOUTPNG=`printf "%s_%02i.png" ${OUTFILEPREFIX} $FRAMEIDX`
  echo "  -> Exporting frame $FRAMEIDX: '$CURRENTLAYERSGROUP@$TMPSVGORIG' -> '$CURRENTOUTPNG'"

  # make a copy of TMPSVG for current frame
  cp $TMPSVG $CURRENTSVG

  # for all layers in CURRENTLAYERSGROUP
  IFS='+'; for LAYERID in $CURRENTLAYERSGROUP; do
    set_layer_visible $CURRENTSVG $LAYERID
  done

  # export CURRENTSVG
  inkscape --export-area-page --without-gui --export-png=$CURRENTOUTPNG $INKSCAPEARG $CURRENTSVG >> /dev/null
  #~ convert $CURRENTOUTPNG -flatten $CURRENTOUTPNG

  # increment frame idx
  FRAMEIDX=$(( $FRAMEIDX + 1 ))
done # for LAYERIDS

#cleaning
rm $TMPSVG*

echo "Done, files generated:"
ls $OUTFILEPREFIX* -alh
