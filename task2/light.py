#!/usr/bin/env python3
"""
Mangol-Tori Rover Autonomy Recruitment - Task 2
Status-light output from rover mission telemetry.

Run:
    python light.py mission_log_1.csv targets.csv

For each mission-log row with a valid timestamp, prints:
    time_s,light

AUTO -> red
MANUAL -> blue
Successful waypoint arrival -> green for 5 seconds
"""

import csv
import math
import sys


EARTH_RADIUS_M = 6_371_000.0
ARRIVAL_DISTANCE_M = 3.0
STOP_SPEED_MPS = 0.5
GREEN_DURATION_S = 5.0


def haversine_distance(lat1, lon1, lat2, lon2):
    """Calculate the great-circle distance between two GPS points in metres."""
    lat1, lon1, lat2, lon2 = map(
        math.radians, (lat1, lon1, lat2, lon2)
    )

    delta_lat = lat2 - lat1
    delta_lon = lon2 - lon1

    a = (
        math.sin(delta_lat / 2.0) ** 2
        + math.cos(lat1) * math.cos(lat2)
        * math.sin(delta_lon / 2.0) ** 2
    )

    # Keep the value in range in case of tiny floating-point rounding errors.
    a = max(0.0, min(1.0, a))
    angle = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))

    return EARTH_RADIUS_M * angle


def find_column(fieldnames, possible_names):
    """Find a column while ignoring capitalization, spaces and underscores."""
    if not fieldnames:
        return None

    columns = {}
    for name in fieldnames:
        if name is not None:
            key = name.strip().lower().replace(" ", "").replace("_", "")
            columns[key] = name

    for candidate in possible_names:
        key = candidate.lower().replace(" ", "").replace("_", "")
        if key in columns:
            return columns[key]

    return None


def valid_position(lat_text, lon_text):
    """Return a valid (latitude, longitude) pair, or None for bad GPS data."""
    try:
        lat = float(lat_text)
        lon = float(lon_text)
    except (TypeError, ValueError):
        return None

    if not math.isfinite(lat) or not math.isfinite(lon):
        return None

    if not (-90.0 <= lat <= 90.0):
        return None
    if not (-180.0 <= lon <= 180.0):
        return None

    return lat, lon


def load_targets(filename):
    """Read waypoint coordinates in the order in which they appear in the CSV."""
    targets = []

    with open(filename, "r", newline="", encoding="utf-8-sig") as file:
        reader = csv.DictReader(file)

        lat_col = find_column(
            reader.fieldnames, ["lat", "latitude", "target_lat"]
        )
        lon_col = find_column(
            reader.fieldnames, ["lon", "lng", "longitude", "target_lon"]
        )

        if lat_col is None or lon_col is None:
            raise ValueError(
                "The targets file must contain latitude and longitude columns."
            )

        for row in reader:
            position = valid_position(row.get(lat_col), row.get(lon_col))
            if position is not None:
                targets.append(position)

    return targets


def process_log(log_filename, targets):
    """Create one status-light result for each row with a usable timestamp."""
    results = []

    previous_position = None
    previous_time = None

    target_index = 0
    green_until = None

    with open(log_filename, "r", newline="", encoding="utf-8-sig") as file:
        reader = csv.DictReader(file)

        time_col = find_column(
            reader.fieldnames, ["time_s", "time", "seconds"]
        )
        mode_col = find_column(reader.fieldnames, ["mode", "state"])
        lat_col = find_column(reader.fieldnames, ["lat", "latitude"])
        lon_col = find_column(
            reader.fieldnames, ["lon", "lng", "longitude"]
        )

        if None in (time_col, mode_col, lat_col, lon_col):
            raise ValueError(
                "The mission log must contain time_s, mode, lat and lon columns."
            )

        for row in reader:
            time_text = (row.get(time_col) or "").strip()

            try:
                current_time = float(time_text)
            except (TypeError, ValueError):
                # Without a timestamp, the required output time is unavailable.
                continue

            if not math.isfinite(current_time):
                continue

            mode = (row.get(mode_col) or "").strip().upper()
            position = valid_position(row.get(lat_col), row.get(lon_col))

            # Estimate speed using the distance between consecutive valid GPS
            # readings divided by the elapsed time between those readings.
            speed = None
            if (
                position is not None
                and previous_position is not None
                and previous_time is not None
            ):
                elapsed = current_time - previous_time
                if elapsed > 0:
                    distance = haversine_distance(
                        previous_position[0], previous_position[1],
                        position[0], position[1]
                    )
                    speed = distance / elapsed

            # The previous waypoint is completed when its 5-second green
            # interval has ended. The next waypoint then becomes active.
            if green_until is not None and current_time >= green_until:
                target_index += 1
                green_until = None

            # Arrival requires all three conditions at the same sample:
            # AUTO mode, within 3 metres, and speed strictly below 0.5 m/s.
            if (
                green_until is None
                and target_index < len(targets)
                and mode == "AUTO"
                and position is not None
                and speed is not None
                and speed < STOP_SPEED_MPS
            ):
                target_lat, target_lon = targets[target_index]
                distance_to_target = haversine_distance(
                    position[0], position[1], target_lat, target_lon
                )

                if distance_to_target <= ARRIVAL_DISTANCE_M:
                    green_until = current_time + GREEN_DURATION_S

            if green_until is not None and current_time < green_until:
                light = "green"
            elif mode == "AUTO":
                light = "red"
            else:
                # MANUAL is blue. Unknown mode labels use the non-AUTO
                # fallback so malformed mode text does not stop the script.
                light = "blue"

            results.append(f"{time_text},{light}")

            # A missing/invalid GPS reading does not erase the last valid
            # position. A non-increasing timestamp is not used for speed.
            if position is not None:
                if previous_time is None or current_time > previous_time:
                    previous_position = position
                    previous_time = current_time

    return results


def main():
    if len(sys.argv) != 3:
        print(
            "Usage: python light.py <mission_log.csv> <targets.csv>",
            file=sys.stderr
        )
        return 1

    log_filename = sys.argv[1]
    targets_filename = sys.argv[2]

    try:
        targets = load_targets(targets_filename)
        output_lines = process_log(log_filename, targets)

        for line in output_lines:
            print(line)

    except (OSError, csv.Error, ValueError) as error:
        print("Error: " + str(error), file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
