import argparse
import geopandas
import shapely
import shapely.geometry
import shapely.wkt
import matplotlib.pyplot


def parse_routes(path, zoom):
    parsed_routes = []
    with open(path) as file:
        lines = [l.strip() for l in file.readlines()]

        for idx, line in enumerate(lines):
            line_coords = [int(i) / zoom for i in line.split()]
            line_coords = [(line_coords[i], line_coords[i + 1]) for i in range(0, len(line_coords), 2)]
            # line_id is needed to plot the routes in different colours.
            parsed_routes.append((idx, shapely.geometry.LineString(line_coords)))

    return geopandas.GeoDataFrame(parsed_routes, columns=['line_id', 'geometry'])


def parse_labels(path, zoom):
    parsed_labels = []
    with open(path) as file:
        lines = [l.strip() for l in file.readlines()]

        for line in lines:
            label_info = line.split()
            x = float(label_info[0]) / zoom
            y = float(label_info[1]) / zoom
            orient = label_info[2]
            parsed_labels.append(get_label(x, y, orient))

    return geopandas.GeoDataFrame(parsed_labels, columns=['geometry'])


def get_label(x, y, orient, width=100, height=50):
    if orient == 'bottom-right':
        return shapely.geometry.Polygon([(x, y), (x + width, y), (x + width, y + height), (x, y + height)])
    elif orient == 'bottom-left':
        return shapely.geometry.Polygon([(x, y), (x, y + height), (x - width, y + height), (x - width, y)])
    elif orient == 'top-left':
        return shapely.geometry.Polygon([(x, y), (x - width, y), (x - width, y - height), (x, y - height)])
    elif orient == 'top-right':
        return shapely.geometry.Polygon([(x, y), (x, y - height), (x + width, y - height), (x + width, y)])
    else:
        raise Exception(f'Invalid label orientation of: {orient}')


def display_results(routes_path, labels_path, zoom_levels):
    for zoom_level in zoom_levels:
        routes = parse_routes(routes_path, zoom=zoom_level)
        labels = parse_labels(labels_path, zoom=zoom_level)

        f, ax = matplotlib.pyplot.subplots()
        routes.plot(ax=ax, column='line_id', linewidth=2)
        labels.plot(ax=ax, facecolor='none', edgecolor='black')
        matplotlib.pyplot.title(f'Results at zoom level {zoom_level}')
        matplotlib.pyplot.gca().invert_yaxis()
        matplotlib.pyplot.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("routes_path", type=str)
    parser.add_argument("labels_path", type=str)
    args = parser.parse_args()

    display_results(args.routes_path, args.labels_path, [1, 2, 4])