
import json
import matplotlib.pyplot as plt
import matplotlib.image as mpimg


def get_path(path, img):
    xs = list()
    ys = list()
    for px in path:
        xs.append(px["i"])
        ys.append(img.shape[0] - px["j"])
    return xs, ys


def get_nodes(g, img):
    xs = list()
    ys = list()
    for node in g:
        xs.append(node["index"]["i"])
        ys.append(img.shape[0] - node["index"]["j"])
    return xs, ys


def run(graph_file, map_file):
    with open(graph_file) as f:
        g = json.loads(f.read())
        img = mpimg.imread(map_file)
        for node in g:
            nbrs = node["neighbours"]
            for nbr in nbrs:
                paths = nbr["paths"]
                for path in paths:
                    xs, ys = get_path(path, img)
                    plt.plot(xs, ys, "r-", linewidth=3, zorder=1)
        xs, ys = get_nodes(g, img)
        plt.scatter(xs, ys, zorder=2, s=100)
        plt.imshow(img, cmap="gray", zorder=0)
        plt.show()


if __name__ == "__main__":
    run("sandbox/graph.json",
        "resources/dynamicvoronoi/strongly_connected.pgm")
