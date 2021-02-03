function getData() {
    var showData = $('#show-data');

    $.getJSON('ramdisk/istro_rt2020_out.json', function (data) {
      //console.log(data);

      showData.empty();
      for (i = 0; i < data.items.length; i++) {
          showData.append(data.items[i] + "<br><br>");
      } 

      showData.append(data.imagen);

      $('#camera-imagefn').empty();
      $('#camera-imagefn').append(data.camera_image);
      if($('#camera-check').prop('checked')){
          $('#camera-image').attr('src', data.camera_image);
          $('#camera-image').attr('class', 'imsize1');
      } else {
          $('#camera-image').attr('src', 'none.gif');
          $('#camera-image').attr('class', 'imsize0');
      }
      $('#vision-imagefn').empty();
      $('#vision-imagefn').append(data.vision_image);
      if($('#vision-check').prop('checked')){
          $('#vision-image').attr('src', data.vision_image);
          $('#vision-image').attr('class', 'imsize1');
      } else {
          $('#vision-image').attr('src', 'none.gif');
          $('#vision-image').attr('class', 'imsize0');
      }
      $('#lidar-imagefn').empty();
      $('#lidar-imagefn').append(data.lidar_image);
      if($('#lidar-check').prop('checked')){
          $('#lidar-image').attr('src', data.lidar_image);
          $('#lidar-image').attr('class', 'imsize1');
      } else {
          $('#lidar-image').attr('src', 'none.gif');
          $('#lidar-image').attr('class', 'imsize0');
      }
      $('#wmgrid-imagefn').empty();
      $('#wmgrid-imagefn').append(data.wmgrid_image);
      if($('#wmgrid-check').prop('checked')){
          $('#wmgrid-image').attr('src', data.wmgrid_image);
          $('#wmgrid-image').attr('class', 'imsize1');
      } else {
          $('#wmgrid-image').attr('src', 'none.gif');
          $('#wmgrid-image').attr('class', 'imsize0');
      }
      $('#navmap-imagefn').empty();
      $('#navmap-imagefn').append(data.navmap_image);
      if($('#navmap-check').prop('checked')){
          $('#navmap-image').attr('src', data.navmap_image);
          $('#navmap-image').attr('class', 'imsize1');
      } else {
          $('#navmap-image').attr('src', 'none.gif');
          $('#navmap-image').attr('class', 'imsize0');
      }
      $('#cdepth-imagefn').empty();
      $('#cdepth-imagefn').append(data.cdepth_image);
      if($('#cdepth-check').prop('checked')){
          $('#cdepth-image').attr('src', data.cdepth_image);
          $('#cdepth-image').attr('class', 'imsize1');
      } else {
          $('#cdepth-image').attr('src', 'none.gif');
          $('#cdepth-image').attr('class', 'imsize0');
      }
    });

    // showData.text('Loading the JSON file.');
}

$(document).ready(function () {
    $.ajaxSetup({ cache: false });
    $('#get-data').click(function () {
        getData();
    });
    setInterval(getData, 1500);
});
