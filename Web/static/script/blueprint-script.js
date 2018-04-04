$(document).ready(function() {



    $("#plus-icon").mouseenter(function(event) {
        $("#plus-icon").addClass('inverted');
    }).mouseout(function(event) {
        $("#plus-icon").removeClass('inverted');
    });

    var input_flag = 0;
    var third_tab_flag = 0;

    var ids = 1;

    //stores the x and y values
    var coords = [];

    var orientations = ["up", "right", "down", "left"]
    var canvas = document.getElementById("map-tab");
    var canvasWidth = canvas.width;
    var canvasHeight = canvas.height;

    var ctx = canvas.getContext("2d");
    //ctx.translate(250, 250);
    ctx.moveTo(canvasWidth/2, canvasHeight/2);

    var draw_map = canvas.getContext("2d");
    draw_map.moveTo(canvasWidth/2, canvasHeight/2);

    var robot_pos = canvas.getContext("2d");
    robot_pos.moveTo(canvasWidth/2, canvasHeight/2);

    var origin = canvas.getContext("2d");
    origin.fillStyle = "#FF0000";

    var squareSize = 7.5;
    origin.fillRect(canvasWidth/2 - squareSize/2 ,canvasHeight/2 - squareSize/2 , squareSize, squareSize);

    var robot_x = 0;
    var robot_y = 0;

    var point_x = 0;
    var point_y = 0;

    var grid = canvas.getContext("2d");

    var p = 0;
    function drawBoard(){
    grid.beginPath();
    grid.strokeStyle = "lightgray";
    for (var x = 0; x <= canvasWidth; x += 50) {
        grid.moveTo(0.5 + x + p, p);
        grid.lineTo(0.5 + x + p, canvasHeight + p);
    }

    for (var x = 0; x <= canvasHeight; x += 50) {
        grid.moveTo(p, 0.5 + x + p);
        grid.lineTo(canvasWidth + p, 0.5 + x + p);
    }
    grid.stroke();
    }

    var move_posX = canvasWidth/2;
    var move_posY = canvasHeight/2;

    drawBoard();


    $("#plus-icon").on('click', function(event) {
        if (input_flag == 0) {
            input_flag = 1;
            $("#draw-button").removeClass('primary');
            $("#draw-button").attr('disabled', 'true');
            $("#finish-button").addClass('primary');
            $("#finish-button").html("Finish");
        }

        var neededId = 'id'.concat((ids-1).toString());

        //capture user input
        var xIN = document.getElementById(neededId).getElementsByClassName('ui input left')[0].children[0].value;
        var yIN = document.getElementById(neededId).getElementsByClassName('ui input right')[0].children[0].value;

        coords.push([xIN, yIN]);

        //draw co-ordinates on canvas map

        ctx.beginPath();
        ctx.moveTo(move_posX, move_posY);
        ctx.strokeStyle = "black";
        ctx.lineTo(canvas.width/2 + Number(xIN), canvas.height/2 - Number(yIN));
        move_posX = canvas.width/2 + Number(xIN);
        move_posY = canvas.height/2 - Number(yIN);
        ctx.stroke();

        var new_locaiton_set = $(".location-set-new").clone(true);
        new_locaiton_set.attr('id', 'id'+ ids++);

        new_locaiton_set.css('display', 'none'); // Fade in later.

        $("#plus-icon").remove();
        if ($(".location-set").length > 4) {
            $("#scrollable-area").css('overflow-y', 'scroll');
        }
        $(".location-set-new").after(new_locaiton_set);
        $(".location-set-new").first().addClass('location-set').removeClass('location-set-new');
        $(".location-set-new > input").val("");
        $(".location-set-new").fadeIn();
        $(".location-set-new > .ui.input").removeClass('disabled');
        $("#plus-icon").removeClass('inverted');
    });


    $("#finish-button").on('click', function(event) {
        if ($("#finish-button").html() == "Skip") {
            $('.ui.basic.modal').modal('show');
        } else {
            $("#first-tab").fadeOut();
            $("#second-tab").fadeIn();
        }


    });

    $("#draw-button").on('click', function(event) {
        third_tab_flag = 1;
        $("#first-tab").fadeOut();
        $("#third-tab").fadeIn();

        //reset canvas add everything else?
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        origin.clearRect(0, 0, canvas.width, canvas.height);

        function drawBoard(){
        grid.beginPath();
        grid.strokeStyle = "lightgray";
        for (var x = 0; x <= canvasWidth; x += 50) {
            grid.moveTo(0.5 + x + p, p);
            grid.lineTo(0.5 + x + p, canvasHeight + p);
        }

        for (var x = 0; x <= canvasHeight; x += 50) {
            grid.moveTo(p, 0.5 + x + p);
            grid.lineTo(canvasWidth + p, 0.5 + x + p);
        }
        grid.stroke();
        }

        drawBoard();

        ctx.translate(canvas.width/2, canvas.height/2);
        var pos = canvas.getContext("2d");
        pos.fillStyle = 'darkblue';

        //var length = 10;
        var $length = document.getElementById('draw input');
        //$length.value = length;

        var x1 = 0;
        var y1 = 0;
        var x2 = 0;
        var y2 = 0;

        var direction = ''

        ctx.fillStyle = 'black';
        draw();

        document.getElementById("btnRight").addEventListener("click", function() {
            if($length.value != '') {
            length = $length.value;
            direction = 'right';
            draw();
            }
        });

        document.getElementById("btnLeft").addEventListener("click", function() {
            if($length.value != '') {
            length = $length.value;
            direction = 'left';
            draw();
            }
        });

        document.getElementById("btnUp").addEventListener("click", function() {
            if($length.value != '') {
            length = $length.value;
            direction = 'up';
            draw();
            }
        });

        document.getElementById("btnDown").addEventListener("click", function() {
            if($length.value != '') {
            length = $length.value;
            direction = 'down';
            draw();
            }
        });

        function draw() {
            if (direction == 'right') {
                ctx.moveTo(x1, y1);
                x2 = x1 + parseInt(length);
                ctx.lineTo(x2, y2);
                //ctx.arc(x2, y2, 40, 0, 2 * Math.PI);
                //pos.translate(x2, y2);
                pos.clearRect(x1, y1, 10, 10);
                pos.fillRect(x2, y2, 10, 10);
                ctx.stroke();
                //pos.setTransform(1, 0, 0, 1, 0, 0);
                x1 = x2;
                y1 = y2;
            }

            if (direction == 'left') {
                ctx.moveTo(x1, y1);
                x2 = x1 - parseInt(length);
                ctx.lineTo(x2, y2);
                //ctx.arc(x2, y2, 40, 0, 2 * Math.PI);
                //pos.translate(x2, y2);
                pos.clearRect(x1, y1, 10, 10);
                pos.fillRect(x2, y2, 10, 10);
                ctx.stroke();
                //pos.setTransform(1, 0, 0, 1, 0, 0);
                x1 = x2;
                y1 = y2;
            }

            if (direction == 'up') {
                ctx.moveTo(x1, y1);
                y2 = y1 - parseInt(length);
                ctx.lineTo(x2, y2);
                //ctx.arc(x2, y2, 40, 0, 2 * Math.PI);
                //pos.translate(x2, y2);
                pos.clearRect(x1, y1, 10, 10);
                pos.fillRect(x2, y2, 10, 10);
                ctx.stroke();
                //pos.setTransform(1, 0, 0, 1, 0, 0);
                x1 = x2;
                y1 = y2;
            }

            if (direction == 'down') {
                ctx.moveTo(x1, y1);
                y2 = y1 + parseInt(length);
                ctx.lineTo(x2, y2);
                //ctx.arc(x2, y2, 40, 0, 2 * Math.PI);
                //pos.translate(x2, y2);
                pos.clearRect(x1, y1, 10, 10);
                pos.fillRect(x2, y2, 10, 10);
                ctx.stroke();
                //pos.setTransform(1, 0, 0, 1, 0, 0);
                x1 = x2;
                y1 = y2;
            }


        };
    });
    $("#confirm-yes").on('click', function(event) {
        $("#first-tab").fadeOut();
        $("#second-tab").fadeIn();
    });

    $("#draw-finish-button").on('click', function(event) {
        $("#third-tab").fadeOut();
        $("#second-tab").fadeIn();
    });


    $("#start-button").on('click', function(event) {
        $.ajax({
            url: '/start-robot',
            type: 'GET',
            success: function() {
                $("#start-button").attr('disabled', 'true');
                $("#start-button").removeClass('green');
                $("#stop-button").addClass('red');
                $("#stop-button").removeAttr('disabled');
		        start_robot();
            },
            error: function() {
                alert("Connection lost. Please examine the bluetooth connection.");
                window.location.href = "/connect.html";
            }
        });
    });

    $("#stop-button").on('click', function(event) {
        $.ajax({
            url: '/stop-robot',
            type: 'GET',
            success: function() {
                $("#stop-button").attr('disabled', 'true');
                $("#stop-button").removeClass('red');
            },
            error: function() {
                alert("Connection lost. Please examine the bluetooth connection.");
            }

        });


    });

    $("#modify-button").on('click', function(event) {
        $("#second-tab").fadeOut();
        if (third_tab_flag == 1) {
            $("#third-tab").fadeIn();
        } else {
            $("#first-tab").fadeIn();
        }

    });


    function start_robot() {


        setInterval(function() {
            $.ajax({
                url: '/newpoints',
                type: 'GET',
                success: function(response) {
                    var json_data = $.parseJSON(response);

                    if(json_data.length==0)
                        return;

                    json_data.forEach(function(item) {

                        if(item.type == "orientation_update")
                        {
                            if(item.new_orientation == "left")
                            {
                                var temp = orientations.pop()
                            }
                        }



                        var x = parseFloat(item.x);
                        var y = parseFloat(item.y);
                        console.log("type: "+item.type+" x: "+item.x+" y: "+item.y);
                        if (item.type == "point") {
                            //dot(x, y);
                            draw_map.beginPath();
                            draw_map.moveTo(point_x, point_y);
                            draw_map.fillStyle = "blue";
                            //draw_map.clearRect(canvas.width/2 + Number(point_x), canvas.height/2 - Number(point_y), 5, 5);
                            draw_map.lineTo(canvas.width/2 + Number(x), canvas.height/2 - Number(y));
                            //draw_map.arc(x, y, 1, 0, 2 * Math.PI, true);
                            draw_map.stroke();
                            point_x = x;
                            point_y = y;

                        } else if (item.type == "self_location") {
                            //line(x, y);
                            robot_pos.beginPath();
                            robot_pos.moveTo(x, y);
                            draw_map.fillStyle = "red";
                            robot_pos.clearRect(canvas.width/2 + Number(robot_x), canvas.height/2 - Number(robot_y), 10, 10);
                            robot_pos.fillRect(canvas.width/2 + Number(x), canvas.height/2 - Number(y), 10, 10);
                            robot_pos.stroke();
                            robot_x = x;
                            robot_y = y;
                        }
                    });
                }
            });

        }, 2000);
    }

    function test(item){
      var x = parseFloat(item.x);
      var y = parseFloat(item.y);
      console.log("type: "+item.type+" x: "+item.x+" y: "+item.y);
      if (item.type == "points") {
          //dot(x, y);
          draw_map.lineTo(canvas.width/2 + Number(x), canvas.height/2 - Number(y), );
          draw_map.stroke();

      } else if (item.type == "self_location") {
          //line(x, y);
          robot_pos.moveTo(x, y);
          robot_pos.clearRect(canvas.width/2 + Number(robot_x), canvas.height/2 - Number(robot_y), 10, 10);
          robot_pos.fillRect(canvas.width/2 + Number(x), canvas.height/2 - Number(y), 10, 10);
          robot_pos.stroke();
          robot_x = x;
          robot_y = y;
      }

    }
});
