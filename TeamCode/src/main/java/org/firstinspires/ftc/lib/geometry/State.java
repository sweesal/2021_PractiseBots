package org.firstinspires.ftc.lib.geometry;

import org.firstinspires.ftc.lib.util.CSVWritable;
import org.firstinspires.ftc.lib.util.Interpolable;

public interface State<S> extends Interpolable<S>, CSVWritable {
    double distance(final S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}
