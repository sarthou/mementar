#include "mementar/core/LtManagement/archiving_compressing/archiving/Archive.h"

#include <cstddef>
#include <iostream>
#include <string>
#include <vector>

#include "mementar/core/LtManagement/archiving_compressing/archiving/Header.h"
#include "mementar/core/LtManagement/archiving_compressing/binaryManagement/BinaryManager.h"
#include "mementar/core/LtManagement/archiving_compressing/compressing/Huffman.h"
#include "mementar/core/LtManagement/archiving_compressing/compressing/LzCompress.h"
#include "mementar/core/LtManagement/archiving_compressing/compressing/LzUncompress.h"

namespace mementar {

  Archive::Archive(const std::string& description, const std::vector<std::string>& files) : BinaryManager("mar"),
                                                                                            description_(description)
  {
    header_.description_file_ = File_t("description");
    for(const auto& file : files)
      header_.input_files_.emplace_back(file);

    // set header_ offset
    size_t header_size = header_.encodedSize();
    header_.description_file_.offset_ = header_size;
    for(auto& file : header_.input_files_)
      file.offset_ = header_size;
  }

  Archive::Archive(const std::string& description, const Header& header) : BinaryManager("mar"),
                                                                           description_(description)
  {
    header_.description_file_ = File_t("description");
    header_.input_files_ = header.input_files_;

    // set header_ offset
    size_t header_size = header_.encodedSize();
    header_.description_file_.offset_ = header_size;
    for(auto& file : header_.input_files_)
      file.offset_ = header_size;
  }

  Archive::Archive() : BinaryManager("mar")
  {}

  std::vector<char> Archive::load()
  {
    size_t offset = 0;

    LzCompress lz_comp;
    std::vector<char> out = lz_comp.compress(description_);
    offset += out.size();
    header_.description_file_.size_ = out.size();

    std::vector<std::string> input_files;
    Huffman huff;
    for(const auto& file : header_.input_files_)
    {
      auto str = huff.readBinaryFile(file.path_);
      if(str.empty())
        std::cout << "can not read file: \'" << file.path_ << "\'" << std::endl;
      input_files.push_back(str);
      huff.analyse(str);
    }

    huff.generateCode();
    std::vector<char> out_vect = huff.getTreeCode();
    out.insert(out.end(), out_vect.begin(), out_vect.end());
    offset += out_vect.size();
    out_vect = std::vector<char>();

    for(size_t i = 0; i < input_files.size(); i++)
    {
      out_vect = huff.getDataCode(input_files[i]);
      out.insert(out.end(), out_vect.begin(), out_vect.end());
      header_.input_files_[i].offset_ += offset;
      offset += out_vect.size();
      header_.input_files_[i].size_ = out_vect.size();
      out_vect = std::vector<char>();
    }

    header_.encode(out_vect);
    out.insert(out.begin(), out_vect.begin(), out_vect.end());

    return out;
  }

  std::vector<char> Archive::load(const std::vector<std::string>& raw_datas)
  {
    if(raw_datas.size() != header_.input_files_.size())
    {
      std::cout << "ERROR" << std::endl;
      return std::vector<char>();
    }

    size_t offset = 0;

    LzCompress lz_comp;
    std::vector<char> out = lz_comp.compress(description_);
    offset += out.size();
    header_.description_file_.size_ = out.size();

    Huffman huff;
    for(const auto& raw_data : raw_datas)
      huff.analyse(raw_data);

    huff.generateCode();
    std::vector<char> out_vect = huff.getTreeCode();
    out.insert(out.end(), out_vect.begin(), out_vect.end());
    offset += out_vect.size();

    out_vect = std::vector<char>();
    for(size_t i = 0; i < raw_datas.size(); i++)
    {
      out_vect = huff.getDataCode(raw_datas[i]);
      out.insert(out.end(), out_vect.begin(), out_vect.end());
      header_.input_files_[i].offset_ += offset;
      offset += out_vect.size();
      header_.input_files_[i].size_ = out_vect.size();
      out_vect = std::vector<char>();
    }

    header_.encode(out_vect);
    out.insert(out.begin(), out_vect.begin(), out_vect.end());

    return out;
  }

  Header Archive::getHeader(const std::vector<char>& data)
  {
    header_.decode(data);
    return header_;
  }

  std::string Archive::extractDescription(const Header& head, const std::vector<char>& data)
  {
    LzUncompress lz;
    std::vector<char> tmp_data(data.begin() + (long)head.description_file_.offset_, data.begin() + (long)head.description_file_.offset_ + (long)head.description_file_.size_);
    return lz.uncompress(tmp_data);
  }

  std::string Archive::extractFile(size_t index, const Header& head, const std::vector<char>& data)
  {
    std::string out;
    Huffman huff;

    std::vector<char> tree_data(data.begin() + (long)head.description_file_.offset_ + (long)head.description_file_.size_, data.end());
    huff.setTree(tree_data);
    std::vector<char> tmp_data(data.begin() + (long)head.input_files_[index].offset_, data.begin() + (long)head.input_files_[index].offset_ + (long)head.input_files_[index].size_);
    out = huff.getFile(tmp_data);

    return out;
  }

} // namespace mementar
