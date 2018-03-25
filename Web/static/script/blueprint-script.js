$(document).ready(function(){
    


    $("#plus-icon").mouseenter(function(event) {
        $("#plus-icon").addClass('inverted');
    }).mouseout(function(event) {
        $("#plus-icon").removeClass('inverted');
    });

    var input_flag = 0;

    
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
        $(".location-set-new").fadeIn();
        $(".location-set-new > .ui.input").removeClass('disabled');
        $("#plus-icon").removeClass('inverted');
    });

    $("#finish-button").on('click', function(event){
        if($("#finish-button").html()=="Skip")
        {
            $('.ui.basic.modal').modal('show');
        }
    });
});