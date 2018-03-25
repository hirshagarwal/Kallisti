$(document).ready(function() {
    $("#ip-button").on('click',  function(event) {
        var ip_address = $("#input-ip").val();
        $("#ip-p").html("Would you like to connect to " + ip_address+"?");
        $('.ui.basic.modal').modal('show');
    });
});