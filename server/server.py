from flask import Flask, current_app, request, render_template, jsonify
from flask_socketio import SocketIO, emit, send

from mapping import Graph, nodes, plottingData

app = Flask(__name__)
app.config['SECRET_KEY'] = 'e928bf8bg802bf7b20ùfb24fg802gf02gf204fu0y2fu'
socketio = SocketIO(app)
mappingGraph = Graph(nodes)


ROBOT_POS: str = ""
PREVIOUS_POSSES: list[str] = []

@app.route("/")
def homepage():
    return render_template("main.html")


@app.route("/robotPos/", methods=["GET", "POST"])
@app.route("/robotPos/<pos>", methods=["GET", "POST"])
def robotPos(pos: str = None):
    global ROBOT_POS, PREVIOUS_POSSES

    # if we have a pos, and the pos is valid..
    if pos and nodes.get(pos):
        ROBOT_POS = pos
        PREVIOUS_POSSES.append(pos)

        return jsonify({"error": False, "pos": ROBOT_POS})

    return jsonify({"error": False, "pos": ROBOT_POS})


# fromTo => .split(":") -> fromTO[0] -> Start fromTo[1] -> End bv: A:AA
# blocks => .split(",") -> [like fromTo] bv: R:S,U:V,
@app.route("/calculatepath/<fromTo>/")
@app.route("/calculatepath/<fromTo>/<blocks>")
def calcPath(fromTo: str, blocks: str = None):
    A,B = fromTo.split(":")
    if blocks:
        blocks = blocks.split(",")
        for block in blocks:
            a,b = block.split(":")
            mappingGraph.BlockConnection(a,b)

    cost, path = mappingGraph.dijkstra(A,B)

    direction_list = []
    for x in range(0, len(path)-1):
        A = path[x]
        B = path[x+1]
        direction_list.append(nodes[A][B][1])


    return jsonify({
        "request": {
            "from": A,
            "to": B,
            "blocks": blocks,
        },
        "path": {
            "cost": cost,
            "path": path,
            "directions": direction_list
        },
    })

@socketio.on('connect')
def test_connect(auth):
    emit('my response', {'data': 'Connected'})


@socketio.on('disconnect')
def test_disconnect():
    print('Client disconnected')


if __name__ == "__main__":
    socketio.run(app, debug=True)

