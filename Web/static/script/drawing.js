var canvas = document.getElementById("tutorial");
var ctx = canvas.getContext("2d");
ctx.translate(240, 240);
var vertices = [];
var counter = 1;
var waypoints;
var last_point;



function calcWaypoints(vertex1, vertex2) {
    var temp_points = [];
    var dx = vertex2.x - vertex1.x;
    var dy = vertex2.y - vertex1.y;
    for (var i = 0; i <= 10; i++) {
        var x = vertex1.x + dx * i / 10;
        var y = vertex1.y + dy * i / 10;
        temp_points.push({ x: x, y: y });
    }
    return temp_points;
}


function drawline(x, y) {

    waypoints=[];
    counter=1;
    if(last_point==undefined)
    {
        last_point={x:x, y:y};
        return 0;
    }
    var points = [];
    points.push(last_point);
    points.push({ x: x, y: y });
    waypoints = calcWaypoints(points[0], points[1]);
    line_animate();
    console.log("not here");
    last_point = points[1];
}


function line_animate() {
    if (counter < waypoints.length - 1) {
        requestAnimationFrame(line_animate);
    }
    console.log("here");
    draw_map.beginPath();
    draw_map.moveTo(waypoints[counter - 1].x, waypoints[counter - 1].y);
    draw_map.lineTo(waypoints[counter].x, waypoints[counter].y);
    draw_map.stroke();
    counter++;

}