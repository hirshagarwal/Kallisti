$(document).ready(function() {



    $("#plus-icon").mouseenter(function(event) {
        $("#plus-icon").addClass('inverted');
    }).mouseout(function(event) {
        $("#plus-icon").removeClass('inverted');
    });

    var input_flag = 0;
    var third_tab_flag = 0;


    $("#plus-icon").on('click', function(event) {
        if (input_flag == 0) {
            input_flag = 1;
            $("#upload-button").removeClass('primary');
            $("#finish-button").addClass('primary');
            $("#finish-button").html("Finish");
        }

        var new_locaiton_set = $(".location-set-new").clone(true);
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