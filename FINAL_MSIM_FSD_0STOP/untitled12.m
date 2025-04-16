if isprop(roadObj, 'Lanes') && ~isempty(roadObj.Lanes)
    laneObj = roadObj.Lanes(1);
    disp(class(laneObj));
    disp(fieldnames(laneObj))
end
