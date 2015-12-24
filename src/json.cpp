
#include "tca/graph.hpp"

/*
 * Converts an Index to JSON
 */
string json_index(Index ind)
{
    stringstream buffer;
    buffer << "{\"i\": " << ind.i << ",";
    buffer << "\"j\": " << ind.j << "}";
    return buffer.str();
}

/*
 * Converts an index and EdgeData to JSON
 */
string json_index_paths(Index ind, EdgeData *ed)
{
    stringstream buffer;
    buffer << "{\"i\": " << ind.i << ",";
    buffer << "\"j\": " << ind.j << ",";
    buffer << "\"paths\": [";
    for (int i = 0; i < ed->paths.size(); i++)
    {
        buffer << "[";
        for (int j = 0; j < ed->paths[i].size(); j++)
        {
            buffer << json_index(ed->paths[i][j]);
            if (j + 1 < ed->paths[i].size())
            {
                buffer << ",";
            }
        }
        buffer << "]";
        if (i + 1 < ed->paths.size())
        {
            buffer << ",";
        }
    }
    buffer << "]}";
    return buffer.str();
}

/*
 * Converts a graph object to JSON
 */
string Graph::json() {
    stringstream buffer;
    buffer << "[";
    int node_c = 0;
    for (auto np : nodes)
    {
        buffer << "{\"index\":" <<  json_index(np.first) << ",";
        buffer << "\"neighbours\": [";
        int nbr_c = 0;
        for (auto nbr : np.second)
        {
            buffer << json_index_paths(nbr, get_edge(np.first, nbr));
            if (++nbr_c < np.second.size())
            {
                buffer << ",";
            }
        }
        buffer << "]}";
        if (++node_c < nodes.size())
        {
            buffer << ",";
        }
    }
    buffer << "]";
    return buffer.str();
}

/*
 * Writes the graph to a file
 */
void Graph::write_to_file(string filename) {
    ofstream out_file;
    out_file.open(filename);
    out_file << this->json();
    out_file.close();
}
