#include <CloudPoints_IO.h>

bool CloudPoints_IO::RootSelector() {
	QFileDialog add_dialog;
	add_dialog.setFileMode(QFileDialog::ExistingFiles);
	q_file_path_ = add_dialog.getOpenFileNames(nullptr, QObject::tr("Select a root."),
		"C:/",
		QObject::tr("All file(*.*);;pcd file (*.pcd);;csv file(*.csv)"));
	if (q_file_path_.isEmpty())
		return(false);

	return(true);
}

bool CloudPoints_IO::csv2pointCloud(QString filepath) {
	QFile csvfile(filepath);
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud < PointXYZRGB>);
	if (csvfile.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		QTextStream infile(&csvfile);
		int Line_i = 0;

		while (!infile.atEnd()) {
			QString lineStr = infile.readLine();
			Line_i++;

			if (Line_i == 1)//欄位名稱跳過				
				continue;

			QStringList res = lineStr.split(",");//分割","

			if (res.size() == 1)
			{
				res = lineStr.split(" ");
			}

			if (res.size() == 1)
			{
				res = lineStr.split("/t");
			}

			PointXYZRGB tmp;
			if (res.size() == 6)
			{
				tmp.x = atof(res[0].toStdString().c_str());
				tmp.y = atof(res[1].toStdString().c_str());
				tmp.z = atof(res[2].toStdString().c_str());
				tmp.r = atof(res[3].toStdString().c_str());
				tmp.g = atof(res[4].toStdString().c_str());
				tmp.b = atof(res[5].toStdString().c_str());
			}
			else
				break;
			cloud->push_back(tmp);
		}
		csvfile.close();

		VoxelGrid<PointXYZRGB> vox;
		vox.setInputCloud(cloud);
		vox.setLeafSize(0.001f, 0.001f, 0.001f);
		vox.filter(*cloud);

		import_cloud_.push_back(cloud);

		if (Line_i <= 1)
			return (false);
		else
			return(true);
	}
}
#include <qdebug.h>
bool CloudPoints_IO::CloudImport() {
	if (!CloudPoints_IO::RootSelector())
		return(false);

	for (int i = 0; i < q_file_path_.size(); i++)
	{
		QFileInfo qfi(q_file_path_[i]);
		if (qfi.suffix() == "csv") {
			file_name_.push_back(qfi.fileName());
			if (CloudPoints_IO::csv2pointCloud(qfi.filePath()))
				continue;
		}
		else if (qfi.suffix() == "pcd") {
			PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud <PointXYZRGB>);
			int importReault = io::loadPCDFile<PointXYZRGB>(qfi.filePath().toLocal8Bit().data(), *cloud);
			if (importReault == 0) {
				file_name_.push_back(qfi.fileName());
				VoxelGrid<PointXYZRGB> vox;
				vox.setInputCloud(cloud);
				vox.setLeafSize(0.001f, 0.001f, 0.001f);
				vox.filter(*cloud);
				import_cloud_.push_back(cloud);
			}
		}
		else
			continue;
	}
	return(true);
}

bool CloudPoints_IO::CloudExport(std::vector<PointCloud<PointXYZRGB>::Ptr> childern_cloudData) {
	QString output_root = QFileDialog::getSaveFileName(nullptr, QObject::tr("Select a root."),
		"C:/", QObject::tr("All file(*.pcd)"));
	if (output_root.isEmpty())
		return(false);

	PointCloud<PointXYZRGBObjectID>::Ptr myObj(new PointCloud < PointXYZRGBObjectID>);
	for (int i = 0; i < childern_cloudData.size(); i++)
	{
		PointCloud<PointXYZRGB>::Ptr tmpcld = childern_cloudData[i]->makeShared();

		for (int j = 0; j < tmpcld->size(); j++)
		{
			PointXYZRGBObjectID tmp;
			tmp.x = tmpcld->points[j].x;
			tmp.y = tmpcld->points[j].y;
			tmp.z = tmpcld->points[j].z;
			tmp.rgb = tmpcld->points[j].rgb;
			tmp.obj_id = i;
			myObj->push_back(tmp);
		}
	}

	QString pcdFileRoot = output_root;
	io::savePCDFile(pcdFileRoot.toStdString(), *myObj);
	return(true);
}
