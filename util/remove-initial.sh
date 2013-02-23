for r in hubo2 huboplus
do

    for f in $OPENHUBO_DIR/robots/$r/*$r*.kinbody.xml
    do
        sed -i '/initial/d' $f
    done
done
