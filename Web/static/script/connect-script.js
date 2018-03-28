$(document).ready(function() {
    $("#ip-button").on('click', function(event) {
        var ip_address = $("#input-ip").val();
        $("#ip-p").html("Would you like to connect to " + ip_address + "?");
        $('.ui.basic.modal').modal('show');
    });

    $("#confirm-yes").on('click', function(event) {
        var ip_format = /[0-9]{1,3}.[0-9]{1,3}.[0-9]{1,3}.[0-9]{1,3}./;
        var ip = $("#input-ip").val()
        if (ip_format.test(ip)) {
            $.ajax({
                url: '/ssh-connection',
                type: 'GET',
                data: { ip_address: $("#input-ip").val() },
                success: function() {
                    window.location.href = "/blueprint.html";
                },
                error: function() {
                    alert("cannot connect to provided IP address");
                }

            });
        }
        else{
            alert("Illegal Ip address.");
        }
    });
});