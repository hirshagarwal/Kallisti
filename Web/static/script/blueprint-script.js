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

    var canvas = document.getElementById("map-tab");
    var canvasWidth = canvas.width;
    var canvasHeight = canvas.height;

    var ctx = canvas.getContext("2d");
    //ctx.translate(250, 250);
    ctx.moveTo(canvasWidth/2, canvasHeight/2);

    var origin = canvas.getContext("2d");
    origin.fillStyle = "#FF0000";
    
    var squareSize = 7.5;
    origin.fillRect(canvasWidth/2 - squareSize/2 ,canvasHeight/2 - squareSize/2 , squareSize, squareSize);


    $("#plus-icon").on('click', function(event) {
        if (input_flag == 0) {
            input_flag = 1;
            $("#upload-button").removeClass('primary');
            $("#finish-button").addClass('primary');
            $("#finish-button").html("Finish");
        }

        var neededId = 'id'.concat((ids-1).toString());

        //capture user input
        var xIN = document.getElementById(neededId).getElementsByClassName('ui input left')[0].children[0].value;
        var yIN = document.getElementById(neededId).getElementsByClassName('ui input right')[0].children[0].value;

        coords.push([xIN, yIN]);

        //draw co-ordinates on canvas map

        ctx.lineTo(canvas.width/2 + Number(xIN), canvas.height/2 - Number(yIN));
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
                    json_data.forEach(function(item) {


                        var re_location = /\((.+?),\s?(.+?)\)/
                        var x = parseFloat(item.location.match(re_location)[1]);
                        var y = parseFloat(item.lcoation.match(re_location)[2]);
                        if (item.type == "points") {
                            dot(x, y);
                        } else if (item.type == "self_location") {
                            line(x, y);
                        }
                    });
                }
            });

        }, 2000);
    }
});