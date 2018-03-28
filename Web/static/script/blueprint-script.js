$(document).ready(function(){
    


    $("#plus-icon").mouseenter(function(event) {
        $("#plus-icon").addClass('inverted');
    }).mouseout(function(event) {
        $("#plus-icon").removeClass('inverted');
    });

    var input_flag = 0;
    var third_tab_flag = 0;

    
    $("#plus-icon").on('click', function(event) {
        if(input_flag == 0){
            input_flag = 1;
            $("#upload-button").removeClass('primary');
            $("#finish-button").addClass('primary');
            $("#finish-button").html("Finish");
        }

        var new_locaiton_set = $(".location-set-new").clone(true);
        new_locaiton_set.css('display', 'none'); // Fade in later.

        $("#plus-icon").remove();
        if($(".location-set").length>4)
        {
            $("#scrollable-area").css('overflow-y', 'scroll');
        }
        $(".location-set-new").after(new_locaiton_set);
        $(".location-set-new").first().addClass('location-set').removeClass('location-set-new');
        $(".location-set-new > input").val("");
        $(".location-set-new").fadeIn();
        $(".location-set-new > .ui.input").removeClass('disabled');
        $("#plus-icon").removeClass('inverted');
    });


    $("#finish-button").on('click', function(event){
        if($("#finish-button").html()=="Skip")
        {
            $('.ui.basic.modal').modal('show');
        }
        else{
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
        $("#start-button").attr('disabled', 'true');
        $("#start-button").removeClass('green');
        $("#stop-button").addClass('red');
        $("#stop-button").removeAttr('disabled');

        console.log("1");

    });

    $("#stop-button").on('click', function(event) {
        $("#stop-button").attr('disabled', 'true');
        $("#stop-button").removeClass('red');
    });

    $("#modify-button").on('click', function(event) {
        $("#second-tab").fadeOut();
        if(third_tab_flag ==1 ){
            $("#third-tab").fadeIn();
        }
        else{
            $("#first-tab").fadeIn();
        }
        
    });
});