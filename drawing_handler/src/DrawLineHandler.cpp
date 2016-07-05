
#include <pluginlib/class_list_macros.h>

#include <drawing_handler/DrawLineHandler.h>
#include <v_repLib.h>
#include <vrep_ros_plugin/access.h>

#include <vrep_ros_plugin/ConsoleHandler.h>


#include <boost/functional/hash.hpp>

const unsigned int DrawLineHandler::DRAWING_DATA_MAIN = boost::hash<std::string>()("DrawLineHandler");
const unsigned int DrawLineHandler::DRAWING_DATA_WIDTH = DrawLineHandler::DRAWING_DATA_MAIN + 1;

const unsigned int DrawLineHandler::DRAWING_DATA_DIFFUSE = DrawLineHandler::DRAWING_DATA_WIDTH + 1;
const unsigned int DrawLineHandler::DRAWING_DATA_SPECULAR = DrawLineHandler::DRAWING_DATA_DIFFUSE + 1;
const unsigned int DrawLineHandler::DRAWING_DATA_EMISSION = DrawLineHandler::DRAWING_DATA_SPECULAR + 1;

const unsigned int DrawLineHandler::DRAWING_DATA_MARKERS = DrawLineHandler::DRAWING_DATA_EMISSION + 1;
const unsigned int DrawLineHandler::DRAWING_DATA_MARKERS_DIFFUSE = DrawLineHandler::DRAWING_DATA_MARKERS + 1;
const unsigned int DrawLineHandler::DRAWING_DATA_MARKERS_SPECULAR = DrawLineHandler::DRAWING_DATA_MARKERS_DIFFUSE + 1;
const unsigned int DrawLineHandler::DRAWING_DATA_MARKERS_EMISSION = DrawLineHandler::DRAWING_DATA_MARKERS_SPECULAR + 1;


DrawLineHandler::DrawLineHandler() : GenericObjectHandler(),
_lastTime(0.0),
_frequency(-1),
_drawingObject(-1),
_width(10){

	for(unsigned int i=0; i<3; ++i){
		_diffuse[i] = _specular[i] = _emission[i] = 0;
	}
	_diffuse[0] = 1.0;

	registerCustomVariables();

}

DrawLineHandler::~DrawLineHandler(){
}

unsigned int DrawLineHandler::getObjectType() const {
	return DRAWING_DATA_MAIN;
}

void DrawLineHandler::synchronize(){
	// We update DrawLineHandler's data from its associated scene object custom data:
	// 1. Get all the developer data attached to the associated scene object (this is custom data added by the developer):
	int buffSize = simGetObjectCustomDataLength(_associatedObjectID, CustomDataHeaders::DEVELOPER_DATA_HEADER);
	char* datBuff=new char[buffSize];
	simGetObjectCustomData(_associatedObjectID, CustomDataHeaders::DEVELOPER_DATA_HEADER,datBuff);
	std::vector<unsigned char> developerCustomData(datBuff,datBuff+buffSize);
	delete[] datBuff;
	// 2. From that retrieved data, try to extract sub-data with the DRAWING_DATA_MAIN tag:
	std::vector<unsigned char> tempMainData;


	if (CAccess::extractSerializationData(developerCustomData,DRAWING_DATA_MAIN,tempMainData)){ // Yes, the tag is there. For now we only have to synchronize _maxVelocity:

		// Remove # chars for compatibility with ROS
		_associatedObjectName = simGetObjectName(_associatedObjectID);
		std::stringstream streamtemp;
		streamtemp << "- [Draw Line] in '" << _associatedObjectName << "'." << std::endl;

	}

}

bool DrawLineHandler::endOfSimulation(){
	if(_drawingObject>0){
		simRemoveDrawingObject(_drawingObject);
		_drawingObject = -1;
	}
	return GenericObjectHandler::endOfSimulation();
}

void DrawLineHandler::handleSimulation(){
	// called when the main script calls: simHandleModule
	if(!_initialized){
		_initialize();
	}

	const simFloat currentSimulationTime = simGetSimulationTime();

	geometry_msgs::PolygonStampedConstPtr line = _line;
	_line.reset(); //only draw new meshes TODO: we might loose a message if it arrives between this and the previous line!

	if ((currentSimulationTime-_lastTime) >= 1.0/_frequency && line != NULL){


		if(_drawingObject>0)
			simRemoveDrawingObject(_drawingObject);
		_drawingObject = simAddDrawingObject(sim_drawing_lines+sim_drawing_painttag+sim_drawing_followparentvisibility, _width, 0.0,
				_associatedObjectID, line->polygon.points.size()-1, _diffuse, NULL, _specular, _emission);

		simFloat data[6];
		for (geometry_msgs::PolygonStamped::_polygon_type::_points_type::const_iterator it = line->polygon.points.begin()+1;
				it!=line->polygon.points.end(); ++it){
			data[0] = (it-1)->x; data[1] = (it-1)->y; data[2] = (it-1)->z;
			data[3] = it->x; data[4] = it->y; data[5] = it->z;
			simAddDrawingObjectItem(_drawingObject, data);
		}


		_lastTime = currentSimulationTime;

	}

}

void DrawLineHandler::_initialize(){
	if (_initialized)
		return;


	std::vector<unsigned char> developerCustomData;
	getDeveloperCustomData(developerCustomData);
	std::vector<unsigned char> tempMainData;

	std::stringstream ss;

	if (CAccess::extractSerializationData(developerCustomData,DRAWING_DATA_MAIN,tempMainData)){
		_frequency=CAccess::pop_float(tempMainData);
		ss << "- [" << _associatedObjectName << "] Drawing frequency set to " << _frequency << "." << std::endl;
	} else {
		_frequency = -1;
		ss << "- [" << _associatedObjectName << "] Drawing frequency not specified. Using simulation step as default." << std::endl;
	}

	if (CAccess::extractSerializationData(developerCustomData,DRAWING_DATA_WIDTH,tempMainData)){
		_width=CAccess::pop_int(tempMainData);
		ss << "- [" << _associatedObjectName << "] Line width set to " << _width << "." << std::endl;
	} else {
		_width = 1;
		ss << "- [" << _associatedObjectName << "] Line width not specified. Using 1 as default" << std::endl;
	}

	if (CAccess::extractSerializationData(developerCustomData,DRAWING_DATA_DIFFUSE,tempMainData)){
		const std::vector<simFloat> tmp = CAccess::pop_float(tempMainData,3);
		if (tmp.size() == 3){
			memcpy(_diffuse, tmp.data(), 3*sizeof(simFloat));
			ss << "- [" << _associatedObjectName << "] Diffuse color set to ["
					<< _diffuse[0] << ", " << _diffuse[1] << ", " << _diffuse[2] << "]." << std::endl;
		}
	} else {
		ss << "- [" << _associatedObjectName << "] Diffuse color not specified. Using ["
				<< _diffuse[0] << ", " << _diffuse[1] << ", " << _diffuse[2] << "] as default" << std::endl;
	}

	if (CAccess::extractSerializationData(developerCustomData,DRAWING_DATA_SPECULAR,tempMainData)){
		const std::vector<simFloat> tmp = CAccess::pop_float(tempMainData,3);
		if (tmp.size() == 3){
			memcpy(_specular, tmp.data(), 3*sizeof(simFloat));
			ss << "- [" << _associatedObjectName << "] Specular color set to ["
					<< _specular[0] << ", " << _specular[1] << ", " << _specular[2] << "]." << std::endl;
		}
	} else {
		ss << "- [" << _associatedObjectName << "] Specular color not specified. Using ["
				<< _specular[0] << ", " << _specular[1] << ", " << _specular[2] << "] as default" << std::endl;
	}

	if (CAccess::extractSerializationData(developerCustomData,DRAWING_DATA_EMISSION,tempMainData)){
		const std::vector<simFloat> tmp = CAccess::pop_float(tempMainData,3);
		if (tmp.size() == 3){
			memcpy(_emission, tmp.data(), 3*sizeof(simFloat));
			ss << "- [" << _associatedObjectName << "] Emission color set to ["
					<< _emission[0] << ", " << _emission[1] << ", " << _emission[2] << "]." << std::endl;
		}
	} else {
		ss << "- [" << _associatedObjectName << "] Emission color not specified. Using ["
				<< _emission[0] << ", " << _emission[1] << ", " << _emission[2] << "] as default" << std::endl;
	}

	std::string topicName(_associatedObjectName);
	std::replace( topicName.begin(), topicName.end(), '#', '_');
	topicName += "/DrawLine";
	_sub = _nh.subscribe(topicName, 1, &DrawLineHandler::lineCallback, this);

	ss << "- [" << _associatedObjectName << "] Waiting for line points on topic " << topicName << "." << std::endl;;
	ConsoleHandler::printInConsole(ss);

	_lastTime = -1e5;
	_initialized=true;
}

void DrawLineHandler::lineCallback(geometry_msgs::PolygonStampedConstPtr msg){
	_line = msg;
}

void DrawLineHandler::registerCustomVariables() const{
#if VREP_VERSION_MAJOR*10000 + VREP_VERSION_MINOR*100 + VREP_VERSION_PATCH < 3*10000 + 3*100 + 1
	simRegisterCustomLuaVariable("sim_ext_ros_bridge_draw_line_data_main", (boost::lexical_cast<std::string>(int(DRAWING_DATA_MAIN))).c_str());

	simRegisterCustomLuaVariable("sim_ext_ros_bridge_draw_line_data_width", (boost::lexical_cast<std::string>(int(DRAWING_DATA_WIDTH))).c_str());

	simRegisterCustomLuaVariable("sim_ext_ros_bridge_draw_line_data_diffuse", (boost::lexical_cast<std::string>(int(DRAWING_DATA_DIFFUSE))).c_str());
	simRegisterCustomLuaVariable("sim_ext_ros_bridge_draw_line_data_specular", (boost::lexical_cast<std::string>(int(DRAWING_DATA_SPECULAR))).c_str());
	simRegisterCustomLuaVariable("sim_ext_ros_bridge_draw_line_data_emission", (boost::lexical_cast<std::string>(int(DRAWING_DATA_EMISSION))).c_str());

	simRegisterCustomLuaVariable("sim_ext_ros_bridge_draw_line_data_markers", (boost::lexical_cast<std::string>(int(DRAWING_DATA_MARKERS))).c_str());
	simRegisterCustomLuaVariable("sim_ext_ros_bridge_draw_line_data_markers_diffuse", (boost::lexical_cast<std::string>(int(DRAWING_DATA_MARKERS_DIFFUSE))).c_str());
	simRegisterCustomLuaVariable("sim_ext_ros_bridge_draw_line_data_markers_specular", (boost::lexical_cast<std::string>(int(DRAWING_DATA_MARKERS_SPECULAR))).c_str());
	simRegisterCustomLuaVariable("sim_ext_ros_bridge_draw_line_data_markers_emission", (boost::lexical_cast<std::string>(int(DRAWING_DATA_MARKERS_EMISSION))).c_str());
#else
	simRegisterScriptVariable("sim_ext_ros_bridge_draw_line_data_main", (boost::lexical_cast<std::string>(int(DRAWING_DATA_MAIN))).c_str(),0);

	simRegisterScriptVariable("sim_ext_ros_bridge_draw_line_data_width", (boost::lexical_cast<std::string>(int(DRAWING_DATA_WIDTH))).c_str(),0);

	simRegisterScriptVariable("sim_ext_ros_bridge_draw_line_data_diffuse", (boost::lexical_cast<std::string>(int(DRAWING_DATA_DIFFUSE))).c_str(),0);
	simRegisterScriptVariable("sim_ext_ros_bridge_draw_line_data_specular", (boost::lexical_cast<std::string>(int(DRAWING_DATA_SPECULAR))).c_str(),0);
	simRegisterScriptVariable("sim_ext_ros_bridge_draw_line_data_emission", (boost::lexical_cast<std::string>(int(DRAWING_DATA_EMISSION))).c_str(),0);

	simRegisterScriptVariable("sim_ext_ros_bridge_draw_line_data_markers", (boost::lexical_cast<std::string>(int(DRAWING_DATA_MARKERS))).c_str(),0);
	simRegisterScriptVariable("sim_ext_ros_bridge_draw_line_data_markers_diffuse", (boost::lexical_cast<std::string>(int(DRAWING_DATA_MARKERS_DIFFUSE))).c_str(),0);
	simRegisterScriptVariable("sim_ext_ros_bridge_draw_line_data_markers_specular", (boost::lexical_cast<std::string>(int(DRAWING_DATA_MARKERS_SPECULAR))).c_str(),0);
	simRegisterScriptVariable("sim_ext_ros_bridge_draw_line_data_markers_emission", (boost::lexical_cast<std::string>(int(DRAWING_DATA_MARKERS_EMISSION))).c_str(),0);
#endif
}

PLUGINLIB_EXPORT_CLASS(DrawLineHandler, GenericObjectHandler)
