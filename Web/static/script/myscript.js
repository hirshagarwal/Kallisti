$(document).ready(function(){
    $("#my-form").on('submit', function(){
        var ip = $("#ip").val();
        var points = $("#points").val();
        var ip_format = /[0-9]{1,3}.[0-9]{1,3}.[0-9]{1,3}.[0-9]{1,3}./;
        if(ip_format.test(ip)){
            return true;
        }
        $("#my-form").after("<p>incorrect format of IP address</p>");
        return false;
    });

    

    setInterval(function(){
        $.ajax({
            url: '/newpoints',
            type: 'GET',
            success: function(response){
                var json_data = $.parseJSON(response);
                json_data.forEach(function(item) {
                    var p = $("<p></p>").text(item.type+" : "+item.location);
                    $("#localizer").after(p);
                    
                    var re_location = /\((.+?),\s?(.+?)\)/
                    var x = parseFloat(item.location.match(re_location)[1]);
                    var y = parseFloat(item.lcoation.match(re_location)[2]);
                    if(item.type=="points"){
                        dot(x, y);
                    }
                    else if (item.type=="self_location")
                    {
                        line(x, y);
                    }
                });
            }
        });
        
    }, 2000);
});




